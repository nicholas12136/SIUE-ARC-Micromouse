import smbus2
import time

bus     = smbus2.SMBus(1)
ADDRESS = 0x14

wait_times = {
    1: 3.0,
    2: 2.0,
    3: 2.0,
    4: 3.0
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
    cmd_names = {1:'move forward', 2:'turn right', 3:'turn left', 4:'turn 180'}
    print(f"Sending: {cmd_names[cmd]}")
    try:
        bus.write_byte_data(ADDRESS, 4, cmd)
    except Exception as e:
        print(f"  I2C write error: {e}")
        return
    time.sleep(wait_times[cmd])
    timeout   = 5
    start     = time.time()
    confirmed = False
    while True:
        s = read_sensors()
        if s is None:
            print("  I2C read error")
        else:
            if not confirmed and s['command'] == cmd:
                confirmed = True
            if s['command'] == 0:
                print("Done.\n")
                return
        if time.time() - start > timeout:
            print("TIMEOUT\n")
            return
        time.sleep(0.1)

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

# ── Menu ──────────────────────────────────────────────────────────────
print("\nRomi Motion Tester")
print("──────────────────")

while True:
    print("Select a motion:")
    print("  1 - Move forward")
    print("  2 - Turn right 90")
    print("  3 - Turn left 90")
    print("  4 - Turn 180")
    print("  q - Quit")

    choice = input("\nEnter choice: ").strip().lower()

    if choice == 'q':
        print("Exiting.")
        break
    elif choice in ('1', '2', '3', '4'):
        wait_for_button("Place robot in position. Press button A to start.")
        send_command(int(choice))
    else:
        print("Invalid choice — enter 1, 2, 3, 4 or q.\n")
