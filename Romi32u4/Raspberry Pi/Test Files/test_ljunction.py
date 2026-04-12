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

# ── L junction test ───────────────────────────────────────────────────
print("\nL Junction Test")
print("───────────────")
print("Place robot at entrance of corridor leading to a right turn.")
input("Press Enter when ready...")

cell = 0

while True:
    s = read_sensors()
    print_sensors(s, f"Cell {cell}")

    if s is None:
        print("Sensor read failed — stopping.")
        break

    # Corridor — keep going straight
    if not s['wallFront'] and s['wallRight']:
        print("Corridor — moving forward.")
        s = send_command(1)
        cell += 1

    # Front wall, right open — turn right
    elif s['wallFront'] and not s['wallRight']:
        print(f"L junction detected after {cell} cells — open right.")
        print("Turning right...")
        send_command(2)
        print("Moving forward into new corridor...")
        s = send_command(1)
        cell += 1
        print_sensors(s, f"Cell {cell} after turn")
        break

    # Front wall, right closed — dead end or left turn
    elif s['wallFront'] and s['wallRight']:
        print(f"Front wall with right closed after {cell} cells.")
        print("No open right passage — check placement.")
        break

    else:
        print("Unexpected reading — stopping.")
        print_sensors(s, "Final")
        break

print("L junction test complete.")
input("Press Enter to exit...")
