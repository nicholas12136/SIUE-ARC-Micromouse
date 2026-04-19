# NRC 2025 — Micromouse Code Archive

This folder contains Arduino sketches and reference files from ARC's 2025 Micromouse competition attempt. These are preserved as a development reference alongside the current year's codebase.

---

## Files

### `ARCBOTV3m.ino`
The primary robot firmware. Implements a full three-phase competition run on a **10×10 maze**:
1. **Explore** — navigates the maze using a flood-fill algorithm, mapping walls via three HC-SR04 ultrasonic sensors (front, left, right) and tracking position with quadrature wheel encoders.
2. **Return** — uses the mapped maze to drive back to the start cell.
3. **Speed Run** — reruns the known optimal path at full motor speed.

Motor control uses an L298N dual H-bridge. Straight-line driving applies a PID correction loop (based on encoder tick delta) during explore/return; speed run disables PID for full throttle. Turns are timed by encoder ticks using a known wheel-base circumference.

**Key constants to calibrate before use:**
- `TICKS_PER_INCH` — encoder ticks per inch of travel
- `turn_circumference` — effective turning diameter (currently 6.8 in)
- Wall detection thresholds in `updateWalls()`

---

### `arduino_mega_ultrasonic_array.ino`
A standalone sensor-test sketch for **four HC-SR04 ultrasonic sensors** on an Arduino Mega. Reads each sensor sequentially with 25 ms inter-sensor delays (to prevent echo cross-talk), converts pulse duration to inches, and prints all four distances over Serial at 9600 baud. Useful for wiring verification and sensor calibration independent of the main robot code.

---

### `mms-stack.ino`
A graph-based, depth-first maze explorer designed for a **16×16 maze**. Represents explored cells as `Node` objects stored in a 2D grid. Each node tracks its walls, traveled exits, and adjacency links to neighboring non-hallway nodes (intersections and turns). A `Vector`-backed stack drives backtracking: the robot pushes newly discovered nodes, explores all open exits, then pops and backtracks when a node is fully exhausted. Intended for use with a maze simulator (e.g., MMS) via stubs for `moveForward()`, `turnLeft()`, `turnRight()`, `wallFront()`, etc.

---

### `pre_edit.txt`
A snapshot of `mms-stack.ino` captured before a round of edits. Kept as a reference to compare against the revised version and recover any logic that may have been changed or removed during editing.