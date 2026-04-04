import smbus2
import time

# ═══════════════════════════════════════════════════════════════
# HARDWARE COMMUNICATION
# ═══════════════════════════════════════════════════════════════

bus     = smbus2.SMBus(1)
ADDRESS = 0x14

# How long to wait after each command before polling for completion
# Conservative values — tune down once everything is stable
WAIT_TIMES = {
    1: 3.0,   # move forward
    2: 2.0,   # turn right
    3: 2.0,   # turn left
    4: 3.0    # turn 180
}

COMMANDS = {
    'move':       1,
    'turn_right': 2,
    'turn_left':  3,
    'turn_180':   4
}

def read_sensors():
    try:
        data = bus.read_i2c_block_data(ADDRESS, 0, 14)
        raw_right = (data[5] << 8) | data[6]
        raw_left  = (data[7] << 8) | data[8]
        enc_left  = (data[9]  << 8) | data[10]
        enc_right = (data[11] << 8) | data[12]
        return {
            'wallFront': bool(data[0]),
            'wallLeft':  bool(data[1]),
            'wallRight': bool(data[2]),
            'command':   data[4],
            'rawRight':  raw_right,
            'rawLeft':   raw_left,
            'encLeft':   enc_left,
            'encRight':  enc_right,
            'buttonA':   bool(data[13])
        }
    except Exception as e:
        print(f"  I2C read error: {e}")
        return None

def send_command(cmd):
    """Send a motion command to the Romi and wait for completion."""
    try:
        bus.write_byte_data(ADDRESS, 4, cmd)
    except Exception as e:
        print(f"  I2C write error: {e}")
        return None

    time.sleep(WAIT_TIMES[cmd])

    timeout = 5
    start   = time.time()

    while True:
        s = read_sensors()
        if s is not None and s['command'] == 0:
            return s
        if time.time() - start > timeout:
            print(f"  TIMEOUT on command {cmd}")
            return None
        time.sleep(0.1)

def move_forward():
    """Command Romi to move forward one cell."""
    return send_command(COMMANDS['move'])

def turn_right():
    """Command Romi to turn right 90 degrees."""
    return send_command(COMMANDS['turn_right'])

def turn_left():
    """Command Romi to turn left 90 degrees."""
    return send_command(COMMANDS['turn_left'])

def turn_180():
    """Command Romi to turn 180 degrees."""
    return send_command(COMMANDS['turn_180'])

def print_sensors(s, label=""):
    """Pretty print sensor state."""
    if s is None:
        print(f"{label} → read failed")
        return
    print(f"{label} → F:{int(s['wallFront'])} "
          f"L:{int(s['wallLeft'])} "
          f"R:{int(s['wallRight'])}")

def wait_for_button(message="Press button A on Romi to continue..."):
    print(message)
    while True:
        s = read_sensors()
        if s is not None and not s['buttonA']:
            break
        time.sleep(0.05)
    while True:
        s = read_sensors()
        if s is not None and s['buttonA']:
            break
        time.sleep(0.05)
    while True:
        s = read_sensors()
        if s is not None and not s['buttonA']:
            break
        time.sleep(0.05)
    print("Starting...")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════
# MAZE DATA STRUCTURES
# ═══════════════════════════════════════════════════════════════

WIDTH  = 10
HEIGHT = 10

# wall_map[x][y] bitmask — same as Main.cpp
# bit 0 = North, bit 1 = East, bit 2 = South, bit 3 = West
wall_map  = [[0]*HEIGHT   for _ in range(WIDTH)]

# distances[x][y] = flood fill distance to goal. 255 = unreachable
distances = [[255]*HEIGHT for _ in range(WIDTH)]

# visited[x][y] = True once robot has physically entered that cell
visited   = [[False]*HEIGHT for _ in range(WIDTH)]

# Robot position and heading
current_x = 0
current_y = 0
heading   = 0  # 0=North 1=East 2=South 3=West

HEADING_NAMES = {0: 'North', 1: 'East', 2: 'South', 3: 'West'}

# Direction vectors — matches Main.cpp
DX = [0,  1, 0, -1]   # North, East, South, West
DY = [1,  0,-1,  0]
DB = [1,  2, 4,  8]   # bitmasks for wall_map

# Goal cells — center 2x2 of 10x10 maze
GOAL_CELLS = [(4,4), (4,5), (5,4), (5,5)]

# Phase tracking
SEARCHING = 0
RETURNING = 1
FAST_RUN  = 2
FINISHED  = 3

current_phase = SEARCHING

# Path storage
exploration_path = []
return_path      = []
fast_path        = []

# ═══════════════════════════════════════════════════════════════
# WALL REGISTRATION
# ═══════════════════════════════════════════════════════════════

def set_wall(x, y, direction):
    """Register a wall at (x,y) in given direction and its neighbor."""
    global wall_map

    wall_map[x][y] |= DB[direction]

    # Also set the wall on the neighboring cell — same as Main.cpp
    if direction == 0 and y < HEIGHT-1: wall_map[x][y+1] |= DB[2]
    if direction == 1 and x < WIDTH-1:  wall_map[x+1][y] |= DB[3]
    if direction == 2 and y > 0:        wall_map[x][y-1] |= DB[0]
    if direction == 3 and x > 0:        wall_map[x-1][y] |= DB[1]

def update_walls(s):
    """Read sensors and register walls around current position."""
    if s['wallFront']: set_wall(current_x, current_y, heading)
    if s['wallLeft']:  set_wall(current_x, current_y, (heading + 3) % 4)
    if s['wallRight']: set_wall(current_x, current_y, (heading + 1) % 4)

def register_boundary_walls():
    """Register the known maze boundary walls at start."""
    set_wall(0, 0, 2)  # south boundary
    set_wall(0, 0, 3)  # west boundary

# ═══════════════════════════════════════════════════════════════
# FLOOD FILL
# ═══════════════════════════════════════════════════════════════

from collections import deque

def flood_fill(targets, visited_only=False):
    """BFS flood fill from target cells outward.
    targets     — list of (x,y) goal cells to fill from
    visited_only — if True, only expand through visited cells
    """
    global distances

    # Reset all distances
    for x in range(WIDTH):
        for y in range(HEIGHT):
            distances[x][y] = 255

    queue = deque()

    # Seed the queue with target cells
    for (tx, ty) in targets:
        if visited_only and not visited[tx][ty]:
            continue
        distances[tx][ty] = 0
        queue.append((tx, ty))

    # BFS outward
    while queue:
        x, y = queue.popleft()
        d    = distances[x][y]

        for direction in range(4):
            nx = x + DX[direction]
            ny = y + DY[direction]

            # Bounds check
            if nx < 0 or nx >= WIDTH or ny < 0 or ny >= HEIGHT:
                continue
            # Wall check
            if wall_map[x][y] & DB[direction]:
                continue
            # Already visited check
            if distances[nx][ny] != 255:
                continue
            # Visited only restriction
            if visited_only and not visited[nx][ny]:
                continue

            distances[nx][ny] = d + 1
            queue.append((nx, ny))

# ═══════════════════════════════════════════════════════════════
# MOVEMENT WRAPPERS
# ═══════════════════════════════════════════════════════════════

def robot_turn_right():
    """Turn right and update heading."""
    global heading
    turn_right()
    heading = (heading + 1) % 4
    print(f"  Turned right — now facing {HEADING_NAMES[heading]}")

def robot_turn_left():
    """Turn left and update heading."""
    global heading
    turn_left()
    heading = (heading + 3) % 4
    print(f"  Turned left — now facing {HEADING_NAMES[heading]}")

def robot_move_forward():
    """Move forward one cell and update position and visited map."""
    global current_x, current_y
    s = move_forward()
    current_x += DX[heading]
    current_y += DY[heading]
    visited[current_x][current_y] = True

    # Record move during searching — moved here from main loop
    if current_phase == SEARCHING:
        exploration_path.append(heading)

    print(f"  Moved to ({current_x}, {current_y}) facing {HEADING_NAMES[heading]}")
    return s

def face_direction(target_dir):
    global heading

    # Check if 180 turn is needed — use dedicated command instead of two 90s
    if (target_dir - heading + 4) % 4 == 2:
        turn_180()
        heading = (heading + 2) % 4
        return

    # Otherwise use 90 degree turns as before
    while heading != target_dir:
        diff = (target_dir - heading + 4) % 4
        if diff == 3:
            robot_turn_left()
        else:
            robot_turn_right()

def detect_initial_heading(s):
    """Detect open passage at start and align heading — same as Main.cpp."""
    global heading
    if not s['wallFront']:
        heading = 0
        print("Open front — heading North.")
    elif not s['wallRight']:
        robot_turn_right()
        print("Open right — heading East.")
    elif not s['wallLeft']:
        robot_turn_left()
        print("Open left — heading West.")
    else:
        print("ERROR: no open passage at start.")
    set_wall(0, 0, (heading + 2) % 4)

# ═══════════════════════════════════════════════════════════════
# PATH TRACING
# ═══════════════════════════════════════════════════════════════

def trace_path(start_x, start_y, visited_only=False):
    """Trace shortest path from start to nearest goal by following
    decreasing distance values. Returns list of directions."""
    path = []
    x, y = start_x, start_y
    limit = WIDTH * HEIGHT

    while distances[x][y] != 0 and len(path) < limit:
        current_dist = distances[x][y]
        moved = False

        for direction in range(4):
            nx = x + DX[direction]
            ny = y + DY[direction]

            if nx < 0 or nx >= WIDTH or ny < 0 or ny >= HEIGHT:
                continue
            if wall_map[x][y] & DB[direction]:
                continue
            if visited_only and not visited[nx][ny]:
                continue
            if distances[nx][ny] < current_dist:
                path.append(direction)
                x, y = nx, ny
                moved = True
                break

        if not moved:
            break

    return path

# ═══════════════════════════════════════════════════════════════
# PATH BUILDING
# ═══════════════════════════════════════════════════════════════

def build_return_path(center_x, center_y):
    """Build path from center back to (0,0) through visited cells."""
    flood_fill([(0, 0)], visited_only=True)

    if distances[center_x][center_y] != 255:
        path = trace_path(center_x, center_y, visited_only=True)
        if path:
            print(f"Return path: smart route, {len(path)} steps.")
            return path

    # Fallback — reverse exploration path
    print("Smart return failed — using reversed exploration path.")
    fallback = [(d + 2) % 4 for d in reversed(exploration_path)]
    print(f"Return path: fallback retrace, {len(fallback)} steps.")
    return fallback

def build_fast_path():
    """Build optimal path from (0,0) to center through visited cells."""
    flood_fill(GOAL_CELLS, visited_only=True)

    if distances[0][0] == 255:
        print("ERROR: no visited-only path from (0,0) to center.")
        return []

    path = trace_path(0, 0, visited_only=True)
    print(f"Fast path ready, {len(path)} steps.")
    return path

# ═══════════════════════════════════════════════════════════════
# SEARCHING NAVIGATION
# ═══════════════════════════════════════════════════════════════

def move_to_best_neighbor():
    """Move to neighboring cell with lowest flood fill distance."""
    best_dir   = -1
    min_score  = 10000

    for direction in range(4):
        if wall_map[current_x][current_y] & DB[direction]:
            continue

        nx = current_x + DX[direction]
        ny = current_y + DY[direction]

        if nx < 0 or nx >= WIDTH or ny < 0 or ny >= HEIGHT:
            continue

        # Prefer lower distance, slight penalty for turning
        score = distances[nx][ny] * 10
        if direction != heading:
            score += 1

        if score < min_score:
            min_score = score
            best_dir  = direction

    if best_dir != -1:
        face_direction(best_dir)
        robot_move_forward()
    else:
        print("WARNING: no valid neighbor found.")

# ═══════════════════════════════════════════════════════════════
# EXECUTE PRECOMPUTED PATH STEP
# ═══════════════════════════════════════════════════════════════

def execute_path_step(path, index):
    """Face the required direction and move forward."""
    next_dir = path[index]
    face_direction(next_dir)
    robot_move_forward()

# ═══════════════════════════════════════════════════════════════
# INITIALIZATION
# ═══════════════════════════════════════════════════════════════

def initialize():
    """Reset all maze state and set up starting conditions."""
    global current_x, current_y, heading, current_phase
    global wall_map, distances, visited
    global exploration_path, return_path, fast_path

    current_x     = 0
    current_y     = 0
    heading       = 0
    current_phase = SEARCHING

    exploration_path = []
    return_path      = []
    fast_path        = []

    for x in range(WIDTH):
        for y in range(HEIGHT):
            wall_map[x][y]  = 0
            distances[x][y] = 255
            visited[x][y]   = False

    visited[0][0] = True
    register_boundary_walls()

    print("Maze initialized.")
    print(f"Start: ({current_x}, {current_y}) facing {HEADING_NAMES[heading]}")

# ═══════════════════════════════════════════════════════════════
# MAIN STATE MACHINE
# ═══════════════════════════════════════════════════════════════

def run():
    """Main maze solving loop."""
    global current_phase, return_path, fast_path
    global exploration_path

    return_path_idx = 0
    fast_path_idx   = 0

    print("\nStarting maze run...")
    print("Press Ctrl+C to abort.\n")

    while current_phase != FINISHED:

        # ── RETURNING ────────────────────────────────────────────
        if current_phase == RETURNING:
            if return_path_idx >= len(return_path):
                print("Back at start. Building fast path...")
                fast_path     = build_fast_path()
                fast_path_idx = 0
                current_phase = FAST_RUN
                print("Starting FAST RUN.")
                continue

            print(f"Return step {return_path_idx + 1}/{len(return_path)}")
            execute_path_step(return_path, return_path_idx)
            return_path_idx += 1
            continue

        # ── FAST RUN ─────────────────────────────────────────────
        if current_phase == FAST_RUN:
            if fast_path_idx >= len(fast_path):
                current_phase = FINISHED
                print("Fast run complete. Mission successful.")
                break

            print(f"Fast step {fast_path_idx + 1}/{len(fast_path)}")
            execute_path_step(fast_path, fast_path_idx)
            fast_path_idx += 1

            # Stop as soon as we enter any goal cell
            for (gx, gy) in GOAL_CELLS:
                if current_x == gx and current_y == gy:
                    current_phase = FINISHED
                    print("Center reached. Mission successful.")
                    break

            continue

        # ── SEARCHING ────────────────────────────────────────────
        s = read_sensors()
        if s is None:
            print("Sensor read failed — aborting.")
            break

        update_walls(s)
        flood_fill(GOAL_CELLS)

        print(f"At ({current_x},{current_y}) "
              f"facing {HEADING_NAMES[heading]} "
              f"dist={distances[current_x][current_y]}")
        print_sensors(s, "  Sensors")

        # Check if center reached
        if distances[current_x][current_y] == 0:
            print("Center reached! Building return path...")
            return_path     = build_return_path(current_x, current_y)
            return_path_idx = 0
            current_phase   = RETURNING
            continue

        move_to_best_neighbor()

# ═══════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("Romi Maze Solver")
    print("════════════════")

    # Verify connection
    print("\nTesting connection...")
    s = read_sensors()
    if s is None:
        print("Connection FAILED — check I2C and button press.")
        exit()
    print("Connection OK.")

    # Initialize maze
    print()
    initialize()

    # Detect starting heading from sensors
    print("\nDetecting initial heading...")
    s = read_sensors()
    detect_initial_heading(s)
    print(f"Ready — facing {HEADING_NAMES[heading]}")

    # Confirm before starting
    print()
    wait_for_button("Place robot in start corner. Press button A to begin.")

    try:
        run()
    except KeyboardInterrupt:
        print("\nAborted by user.")
        send_command  # motors already stopped by Romi
    finally:
        print(f"\nFinal position: ({current_x}, {current_y})")
        print(f"Final heading:  {HEADING_NAMES[heading]}")
        print(f"Cells visited:  {sum(visited[x][y] for x in range(WIDTH) for y in range(HEIGHT))}")
        print(f"Phase reached:  {['SEARCHING','RETURNING','FAST_RUN','FINISHED'][current_phase]}")
