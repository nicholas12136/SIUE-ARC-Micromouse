# Simulation

This folder contains the tools used to validate the flood fill navigation logic before deploying it to the physical robot. It is split into two subfolders: **Flood Fill Logic** and **Mazes**.

---

## Flood Fill Logic

Contains the mouse algorithm and its simulator interface, designed to run inside the [mms Micromouse Simulator](https://github.com/mackorone/mms) for visual debugging.

| File | Purpose |
| :--- | :--- |
| `Main.cpp` | Core navigation algorithm — flood fill logic, wall map, state machine, and path computation. |
| `API.cpp` | Simulator communication layer — translates C++ function calls into mms stdin/stdout text commands. |
| `API.h` | Declarations for the API layer. |

### How it works with mms

mms communicates with the algorithm via stdin/stdout. `API.cpp` is the bridge: when `Main.cpp` calls `API::wallFront()`, it sends the text command `wallFront` to mms and reads back `true` or `false`. Motion commands like `moveForward` and `turnRight` work the same way. mms renders the maze visually, updating flood fill numbers and wall markings in real time as the mouse navigates.

Use mms when you need to **visually observe** the mouse's behavior on a specific maze — watching how the flood fill values update, how walls get registered, and how each phase transition occurs.

### Compilation

Compile from the `Flood Fill Logic` directory using MSYS2 MINGW64:

```bash
g++ Main.cpp API.cpp -o mouse.exe -std=c++17 -static-libgcc -static-libstdc++
```

The `-static` flags bundle the runtime libraries into the executable so it runs correctly when spawned outside the MSYS2 environment.

### Navigation Phases

`Main.cpp` solves the maze in three sequential phases controlled by the `currentState` variable:

1. **SEARCHING** — Explores the maze in real time, registering walls and re-running flood fill after every move to always navigate toward the goal via the best known path.
2. **RETURNING** — Once the center is reached, computes the shortest path back to `(0,0)` through visited cells. Falls back to retracing the exploration path in reverse if no shortcut exists.
3. **FAST_RUN** — Executes the pre-computed optimal path from `(0,0)` to the center without any sensor checks. This is the timed competition run.

### Wall Map

`wall_map[x][y]` is a 4-bit bitmask encoding walls around each cell:

| Bit | Value | Direction |
| :--- | :--- | :--- |
| 0 | 1 | North |
| 1 | 2 | East |
| 2 | 4 | South |
| 3 | 8 | West |

Every wall set on one cell automatically sets the matching wall on the adjacent cell (symmetry enforcement).

---

## Mazes

Contains the maze generation and automated testing tools.

| File | Purpose |
| :--- | :--- |
| `generate_maze.py` | Generates valid, competition-accurate `.maz` maze files. |
| `test_harness.py` | Compiles `mouse.exe` and runs it headlessly against every `.maz` file in this folder. |
| `*.maz` | Generated maze files used for testing. A reference template is included to illustrate the file format. |

### Maze File Format (`.maz`)

Each line represents one cell:

```
x y n e s w
```

Where `x`, `y` are the cell coordinates and `n`, `e`, `s`, `w` are `1` (wall present) or `0` (open) for North, East, South, West respectively. The file contains one line per cell ordered by `x` then `y`, with `(0,0)` at the bottom-left.

**Example:** `3 5 1 0 0 1` — cell (3,5) has a North wall and a West wall.

### Maze Generation (`generate_maze.py`)

Generates mazes in four stages:

1. **Prim's Algorithm** — Produces a fully connected random maze where every cell is reachable.
2. **Center Enforcement** — Clears internal walls of the 2×2 center area `(4,4)–(5,5)` and seals the perimeter with exactly one entrance, matching NRC contest rules.
3. **Start Enforcement** — Ensures `(0,0)` has exactly three walls with one random open passage (North or East).
4. **Connectivity Validation** — Flood fills from `(0,0)` to confirm every cell is still reachable. Retries from scratch if not.

**Key functions:**

| Function | Description |
| :--- | :--- |
| `generate_maze(width, height)` | Runs all four stages and returns valid wall arrays. |
| `generate_and_save_maze(filename)` | Generates one maze and writes it to disk. |
| `generate_batch(count, prefix, target_dir)` | Generates multiple maze files, named `prefix_001.maz`, `prefix_002.maz`, etc. |

### Test Harness (`test_harness.py`)

Runs `mouse.exe` headlessly against every `.maz` file — no graphical interface. Python spawns the executable as a subprocess and speaks the same stdin/stdout protocol as mms, so the algorithm never knows the difference.

**What it does:**

1. Optionally generates fresh mazes via `generate_maze.py`.
2. Compiles `mouse.exe` using g++.
3. Simulates mms responses (wall queries, move acknowledgements) based on the loaded maze data.
4. Tracks position, detects crashes, and enforces a 5000-step limit.
5. Reports pass/fail with step counts and copies failed mazes to a `failures/` folder for mms inspection.

A run counts as a **pass** only if the mouse completes all three phases — reaches the center, returns to `(0,0)`, and reaches the center again on the fast run.

**Run from Command Prompt** (not MSYS2) from the `mms` root directory:

```bash
python test_harness.py
```

**Current metrics:** 100% pass rate across 100 mazes, averaging 68 steps per full run.
