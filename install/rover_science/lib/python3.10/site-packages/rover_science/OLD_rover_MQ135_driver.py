# mq135_ads1115.py
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class MQ135:
    def __init__(self, channel, r_load=10000, r_zero=76000):
        self.channel = channel
        self.RLOAD = r_load      # Load resistance in ohms
        self.RZERO = r_zero      # Sensor resistance in clean air
        self.VCC = 5.0           # Circuit voltage
        
        # MQ-135 calibration constants (adjust for target gas)
        self.PARA = 116.602068   # CO2 coefficient
        self.PARB = -2.769034    # CO2 exponent

    def read_voltage(self):
        """Read raw voltage from ADC channel"""
        return self.channel.voltage

    def calculate_ppm(self):
        """Convert voltage reading to PPM value"""
        voltage = self.read_voltage()
        
        # Avoid division by zero
        if voltage < 0.01:
            voltage = 0.01
            
        rs = (self.VCC / voltage - 1) * self.RLOAD
        ratio = rs / self.RZERO
        return self.PARA * (ratio ** self.PARB)

    def calibrate_clean_air(self):
        """Calibrate RZERO in clean air (run in fresh outdoor air)"""
        avg_voltage = sum([self.read_voltage() for _ in range(100)]) / 100
        self.RZERO = (self.VCC / avg_voltage - 1) * self.RLOAD
        return self.RZERO

# Initialize I2C and ADS1115
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
adc_channel = AnalogIn(ads, ADS.P0)  # Connect sensor AO to ADS1115 A0

# Create MQ135 instance
mq135 = MQ135(adc_channel)

try:
    print("MQ135 Warming Up... (15 seconds)")
    time.sleep(15)  # Initial warm-up
    
    while True:
        voltage = mq135.read_voltage()
        ppm = mq135.calculate_ppm()
        
        print(f"Voltage: {voltage:.3f} V | Estimated CO₂: {ppm:.1f} ppm")
        time.sleep(1)

except KeyboardInterrupt:
    print("\nSensor monitoring stopped")
