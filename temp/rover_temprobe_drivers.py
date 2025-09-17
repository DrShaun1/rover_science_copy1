import spidev
import time

# --- SPI Setup ---
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 → /dev/spidev0.0
spi.max_speed_hz = 500000  # Safe speed
spi.mode = 0b00  # SPI mode 0

def read_max6675():
    # Just read the data; no need to toggle CS because it's tied low
    raw = spi.xfer2([0x00, 0x00])
    value = (raw[0] << 8) | raw[1]

    print(f"Raw SPI: {raw} → 0x{value:04X} → {bin(value)}")

    if value & 0x04:
        return None  # Thermocouple disconnected

    temp_c = (value >> 3) * 0.25
    return temp_c

try:
    while True:
        temp = read_max6675()
        if temp is not None:
            print(f"Temperature: {temp:.2f} °C")
        else:
            print("⚠️ Thermocouple not connected or open circuit!")
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    spi.close()
