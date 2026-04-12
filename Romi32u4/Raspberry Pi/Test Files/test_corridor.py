import smbus2
import time

bus     = smbus2.SMBus(1)
ADDRESS = 0x14

wait_times = {1: 3.0, 2: 2.0, 3: 2.0, 4: 3.0}

def read_sensors():
    try:
        data = bus.read_i2c_block_data(ADDRESS, 0, 13)
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
            'encRight':  enc_right
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
    print(f"{label} → "
          f"F:{int(s['wallFront'])} "
          f"L:{int(s['wallLeft'])}(raw:{s.get('rawLeft','?')}) "
          f"R:{int(s['wallRight'])}(raw:{s.get('rawRight','?')}) "
          f"encL:{s.get('encLeft','?')} "
          f"encR:{s.get('encRight','?')})")

# ── Wall centering test ───────────────────────────────────────────────
print("\nWall Centering Test")
print("───────────────────")
print("Place robot at start of corridor — try different angles.")
print("Robot runs until front wall detected.")
input("Press Enter when ready...")

cell = 0

while True:
    s = read_sensors()
    print_sensors(s, f"Cell {cell}")

    if s is None:
        print("Sensor read failed — stopping.")
        break

    # Only stop for front wall — ignore side sensor state
    if s['wallFront']:
        print(f"Front wall detected after {cell} cells — stopping.")
        break

    print("Moving forward.")
    s = send_command(1)
    cell += 1

print(f"\nRun complete — {cell} cells driven.")
input("Press Enter to exit...")
