import smbus2
import time

bus     = smbus2.SMBus(1)
ADDRESS = 0x14

wait_times = {1: 3.0, 2: 2.0, 3: 2.0, 4: 3.0}

def read_sensors():
    try:
        data = bus.read_i2c_block_data(ADDRESS, 0, 5)
        return {
            'wallFront': bool(data[0]),
            'wallLeft':  bool(data[1]),
            'wallRight': bool(data[2]),
            'command':   data[4]
        }
    except Exception as e:
        print(f"  I2C read error: {e}")
        return None

def send_command(cmd):
    try:
        bus.write_byte_data(ADDRESS, 4, cmd)
    except Exception as e:
        print(f"  I2C write error: {e}")
        return None

    time.sleep(wait_times[cmd])

    timeout = 5
    start   = time.time()

    while True:
        s = read_sensors()
        if s is not None and s['command'] == 0:
            return s
        if time.time() - start > timeout:
            print("TIMEOUT")
            return None
        time.sleep(0.1)

def print_sensors(s, label=""):
    if s is None:
        print(f"{label} → read failed")
        return
    print(f"{label} → F:{int(s['wallFront'])} L:{int(s['wallLeft'])} R:{int(s['wallRight'])}")

def decide_turn(s):
    # Returns turn command based on sensor reading
    # Right open
    if s['wallFront'] and not s['wallRight']:
        return 2, "right"
    # Left open
    elif s['wallFront'] and not s['wallLeft']:
        return 3, "left"
    # Dead end
    elif s['wallFront'] and s['wallRight'] and s['wallLeft']:
        return 4, "180"
    else:
        return None, "unknown"

# ── U shape test ──────────────────────────────────────────────────────
print("\nU Shape Test")
print("────────────")
print("Place robot at entrance of U shaped corridor.")
print("Two turns in the same direction leading to parallel corridor.")
input("Press Enter when ready...")

cell     = 0
turns    = 0

while True:
    s = read_sensors()
    print_sensors(s, f"Cell {cell}")

    if s is None:
        print("Sensor read failed — stopping.")
        break

    # Corridor — keep going straight
    if not s['wallFront'] and s['wallRight']:
        print("Corridor — moving forward.")
        send_command(1)
        cell += 1

    # Junction or dead end
    elif s['wallFront']:
        cmd, direction = decide_turn(s)

        if cmd is None:
            print("Unknown junction — stopping.")
            print_sensors(s, "Final")
            break

        print(f"Junction detected — turning {direction}.")
        send_command(cmd)
        turns += 1

        # Move one cell into new corridor after turn
        print("Moving into new corridor...")
        s = send_command(1)
        cell += 1
        print_sensors(s, f"Cell {cell} after turn {turns}")

        # After two turns we've completed the U
        if turns == 2:
            print(f"\nU shape complete — {cell} total cells, 2 turns.")
            break

    else:
        print("Unexpected reading — stopping.")
        print_sensors(s, "Final")
        break

print("U shape test complete.")
input("Press Enter to exit...")
