# Micromouse — PlatformIO Project

This is a single PlatformIO project with two build environments:

| Environment | Target ESP32 | How it's used |
|-------------|-------------|---------------|
| `receiver`  | Connected to your PC via USB | Receives ESP-NOW packets, prints telemetry to serial monitor |
| `mouse`     | Mounted on the micromouse | Runs motor/sensor tests and sends data wirelessly via ESP-NOW |

## Project Structure
```
CompleteMouseTestReceiver/
├── platformio.ini
├── README.md
└── src/
    ├── shared/
    │   └── mouse_packet.h   # ESP-NOW data struct (included by both envs)
    ├── mouse/
    │   └── main.cpp         # Mouse ESP32 firmware
    └── receiver/
        └── main.cpp         # Receiver ESP32 firmware
```

## VS Code Workspace Setup

Open **only** the `CompleteMouseTestReceiver` folder in VS Code (or in your
`.code-workspace` file). The `CompleteMouseTestSender` folder is no longer
needed — all firmware now lives in this single project.

PlatformIO will detect both `[env:mouse]` and `[env:receiver]` automatically.
Use the PlatformIO toolbar at the bottom of VS Code to switch between them,
or use the CLI commands below.

## First-Time Setup

### Step 1 — Flash the receiver
1. Connect the receiver ESP32 to your PC via USB
2. In VS Code open the PlatformIO sidebar → select `receiver` environment → **Upload**
   Or via CLI:
   ```
   pio run -e receiver -t upload
   ```
3. Open the serial monitor (115200 baud):
   ```
   pio device monitor -e receiver
   ```
4. Note the MAC address printed on boot, e.g. `24:6F:28:AA:BB:CC`

### Step 2 — Set the receiver MAC in mouse firmware
Open `src/mouse/main.cpp` and update:
```cpp
uint8_t RECEIVER_MAC[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};
```

### Step 3 — Select a test mode
In `src/mouse/main.cpp`, uncomment exactly ONE:
```cpp
#define TEST_STRAIGHT   // velocity PID straight line
#define TEST_TURN       // encoder-based left turn
#define TEST_SENSORS    // IR sensor readings
#define TEST_ESPNOW     // ESP-NOW link check only
```

### Step 4 — Flash the mouse
Disconnect the receiver, connect the mouse ESP32, then:
```
pio run -e mouse -t upload
```

### Step 5 — Monitor
Reconnect the receiver. The serial monitor will print live telemetry from the
mouse as it runs the selected test.

## Test Modes

| Mode | What it does | Runs in |
|------|-------------|---------|
| `TEST_STRAIGHT` | Drives forward 2s with velocity PID, sends M1/M2 counts + PWM | `setup()` |
| `TEST_TURN` | Executes `turn90()`, sends encoder counts during turn | `setup()` |
| `TEST_SENSORS` | Reads IR sensors every 100ms, sends distances + front state | `loop()` |
| `TEST_ESPNOW` | Sends incrementing counter packets every 500ms (no motors) | `loop()` |

## Tuning Parameters (`src/mouse/main.cpp`)

**Straight:**
- `M1_TARGET`, `M2_TARGET` — target encoder counts per 10ms
- `kP`, `kI`, `kD` — PID gains

**Turn:**
- `COUNTS_PER_90` — encoder counts for a 90° left turn
- `M1_TURN_PWM`, `M2_TURN_PWM` — starting PWM for each motor

**Sensors:**
- `voltageToDistanceCm()` — empirical model, adjust coefficients if readings drift
