import smbus2
import time

bus     = smbus2.SMBus(1)
ADDRESS = 0x14

print("Watching button state — press button A on Romi...")
print("Ctrl+C to stop")
print()

while True:
    try:
        data = bus.read_i2c_block_data(ADDRESS, 0, 14)
        button = bool(data[13])
        print(f"buttonA: {int(button)}  (raw byte: {data[13]})")
    except Exception as e:
        print(f"I2C error: {e}")
    time.sleep(0.1)
