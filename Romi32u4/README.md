# Romi32u4

This folder contains the hardware-side implementation of the maze solver. The system is split across two components that communicate over I²C: the **Romi 32U4** handles all low-level motor control and sensor reading, while the **Raspberry Pi** runs the navigation logic and issues commands.

---

## PololuRpiSlave.ino

The Arduino sketch flashed to the Romi 32U4 control board. It acts as an I²C slave, exposing a shared data buffer that the Raspberry Pi reads from and writes to.

### Shared Data Buffer (`struct Data`)

The I²C interface is built around a single struct that both sides of the connection read and write:

| Field | Direction | Description |
| :--- | :--- | :--- |
| `wallFront` | Romi → Pi | `1` if front IR sensor detects a wall |
| `wallLeft` | Romi → Pi | `1` if left IR sensor detects a wall |
| `wallRight` | Romi → Pi | `1` if right IR sensor detects a wall |
| `moveDone` | Romi → Pi | `1` when the last command has finished executing |
| `command` | Pi → Romi | Motion command to execute (see table below) |
| `rawLeftHigh/Low` | Romi → Pi | Raw analog value from left IR sensor (split across 2 bytes) |
| `rawRightHigh/Low` | Romi → Pi | Raw analog value from right IR sensor (split across 2 bytes) |
| `encLeftHigh/Low` | Romi → Pi | Left encoder count (split across 2 bytes) |
| `encRightHigh/Low` | Romi → Pi | Right encoder count (split across 2 bytes) |
| `buttonA` | Romi → Pi | `1` while Button A is pressed |

### Command Table

| Value | Command |
| :--- | :--- |
| `1` | Move forward one cell |
| `2` | Turn right 90° |
| `3` | Turn left 90° |
| `4` | Turn 180° |

### Sensor Configuration

| Sensor | Pin | Type |
| :--- | :--- | :--- |
| Front IR | D11 | Digital (LOW = wall detected) |
| Left IR | A2 | Analog (threshold: 200) |
| Right IR | A3 | Analog (threshold: 200) |

### Motion Primitives

All motion functions are encoder-driven and follow a ramp-up → cruise → ramp-down speed profile to reduce wheel slip and overshoot.

**`moveOneCell()`**
Drives forward one maze cell. During all three speed phases it applies two concurrent correction terms:
- *Encoder correction* (`KP_ENC = 0.65`) — keeps left and right wheel counts equal to drive straight.
- *Wall centering* (`KP_WALL = 0.05`, `KD_WALL = 0.25`) — when walls are present on both sides, balances the analog IR readings to center the robot in the corridor.

The front IR sensor is checked at every step. If a wall is detected mid-move, the robot stops immediately rather than completing the cell.

**`turnRight90()` / `turnLeft90()`**
Spins in place using opposite motor directions until the encoder counts reach `TURN_TARGET = 630`. A 200ms pause follows each turn to let the robot settle.

**`turnRight180()`**
Same as a 90° turn but drives to `TURN180_TARGET = 1350`.

### Tuning Constants

| Constant | Value | Purpose |
| :--- | :--- | :--- |
| `BASE_SPEED` | 200 | Cruise speed (motor PWM units) |
| `TURN_SPEED` | 150 | Speed used during turns |
| `CELL_TARGET` | 1000 | Encoder counts per cell |
| `TURN_TARGET` | 650 | Encoder counts for 90° turn |
| `TURN180_TARGET` | 1375 | Encoder counts for 180° turn |
| `RAMP_STEP` | 6 | Speed increment per ramp tick |
| `RAMP_DELAY` | 10ms | Delay between ramp ticks |
| `DECEL_START` | 100 | Encoder counts before target to begin deceleration |

### Setup & Loop

`setup()` initializes serial, configures the front IR pin, starts the I²C slave at address `0x14`, and waits for Button A to be pressed before signaling ready. This gives the operator time to position the robot before it accepts commands.

`loop()` continuously updates the shared buffer with fresh sensor and encoder readings, then checks for an incoming command. When a non-zero command is received, `moveDone` is cleared, the appropriate motion function is called, and `moveDone` is set back to `1` when complete.

---

## Raspberry Pi

### `maze.py`

The main navigation script. It runs the complete three-phase maze solver on the Pi and communicates with the Romi over I²C using the `smbus2` library.

#### I²C Communication

| Function | Description |
| :--- | :--- |
| `read_sensors()` | Reads all 14 bytes from the Romi buffer and returns a dict of wall states, raw IR values, encoder counts, and button state. |
| `send_command(cmd)` | Writes a command byte to the buffer, waits a fixed time for the motion to start, then polls until `command == 0` (Romi signals completion). 5-second timeout. |
| `move_forward()` / `turn_right()` / `turn_left()` / `turn_180()` | Convenience wrappers around `send_command`. |

#### State Machine

The solver runs through four states in sequence — transitions are one-way and never reversed:

```
SEARCHING → RETURNING → FAST_RUN → FINISHED
```

| State | Behavior |
| :--- | :--- |
| `SEARCHING` | Reads sensors, registers walls, re-runs flood fill, moves to the lowest-distance neighbor. Repeats until a goal cell is reached. |
| `RETURNING` | Executes the pre-computed return path step by step back to `(0,0)`. Tries a smart shortest route through visited cells first; falls back to reversing the exploration path if that fails. |
| `FAST_RUN` | Executes the pre-computed optimal path to the center without any sensor checks. Ends when a goal cell is entered. |
| `FINISHED` | Mission complete — prints final position, heading, cells visited, and phase reached. |

#### Flood Fill (`flood_fill`)

BFS from a set of target cells outward. Assigns each cell a distance value representing the shortest number of moves to the nearest target given the currently known wall map. Accepts a `visited_only` flag that restricts expansion to cells the robot has physically entered — used for return path and fast path computation to avoid routing through cells with unknown wall data.

#### Wall Map

`wall_map[x][y]` is a 4-bit bitmask (same convention as `Main.cpp`): bits 0–3 represent North, East, South, West walls. `set_wall()` enforces symmetry — setting a wall on one cell automatically sets the matching wall on the neighbor.

#### Key Functions

| Function | Description |
| :--- | :--- |
| `update_walls(s)` | Registers front, left, and right walls from a sensor reading at the current position. |
| `move_to_best_neighbor()` | Picks the open neighbor with the lowest flood fill distance, with a small penalty (+1) for changing direction to avoid unnecessary turns. |
| `build_return_path(cx, cy)` | Computes the path from the center back to `(0,0)`. Smart route first, fallback retrace second. |
| `build_fast_path()` | Runs flood fill from goal cells over visited-only cells and traces the shortest path from `(0,0)`. |
| `detect_initial_heading(s)` | Reads sensors at startup to find the one open passage and orients the coordinate system accordingly. |
| `face_direction(target_dir)` | Turns the robot to face a target heading using the minimum number of turns, using `turn_180()` when the target is directly behind. |
| `initialize()` | Resets all state — wall map, distances, visited flags, paths, position, and heading. |

#### Entry Point

On launch, `maze.py`:
1. Verifies the I²C connection to the Romi.
2. Initializes the maze state and detects the initial heading from sensor readings.
3. Waits for Button A to be pressed before starting (gives time to place the robot).
4. Runs the state machine loop until `FINISHED` or a keyboard interrupt.

---

### Test Files

Standalone scripts for verifying hardware behavior in isolation before running the full solver. Each connects directly to the Romi over I²C and tests a specific scenario.

| File | What it tests |
| :--- | :--- |
| `test_romi.py` | Basic I²C connection and a single forward move command. |
| `test_all_moves.py` | All four motion commands in sequence. |
| `test_button_press.py` | Button A detection via the shared buffer. |
| `test_corridor.py` | Straight-line driving through a corridor with wall centering active. |
| `test_deadend.py` | Dead-end detection and 180° turn response. |
| `test_ljunction.py` | Behavior at an L-shaped junction (one turn required). |
| `test_ushape.py` | U-shape navigation (forward, dead-end, 180°, forward). |

Run any test file on the Pi via SSH:

```bash
python3 test_<scenario>.py
```

Make sure the Romi is powered on and Button A has been pressed (Romi ready signal) before running a test.
