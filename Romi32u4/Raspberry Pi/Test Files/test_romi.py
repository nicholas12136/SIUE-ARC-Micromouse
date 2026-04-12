import smbus2
import time

bus     = smbus2.SMBus(1)
ADDRESS = 0x14

def read_sensors():
    data = bus.read_i2c_block_data(ADDRESS, 0, 5)
    return {
        'wallFront': bool(data[0]),
        'wallLeft':  bool(data[1]),
        'wallRight': bool(data[2]),
        'moveDone':  bool(data[3])
    }

def send_command(cmd):
    bus.write_byte_data(ADDRESS, 4, cmd)
    # Wait until Romi confirms the move is done
    time.sleep(0.1)
    while not read_sensors()['moveDone']:
        time.sleep(0.05)
    print("Move complete.")

print("Sending move forward command in 3 seconds...")
print("Make sure the robot has space in front of it!")
time.sleep(3)

send_command(1)  # 1 = moveForward one cell
print("Done.")
