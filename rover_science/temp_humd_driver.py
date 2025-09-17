import smbus2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# I2C address of the SHT30 sensor (default is 0x44)
SHT30_ADDRESS = 0x44

# I2C bus number (check with 'ls /dev/i2c-*')
I2C_BUS = 1

# SHT30 commands
MEASUREMENT_COMMAND = [0x2C, 0x06]

class TempHumidityPublisher(Node):
    def __init__(self):
        super().__init__('temp_humidity_publisher')
        self.temp_publisher = self.create_publisher(Float32, 'temperature_data', 10)
        self.humidity_publisher = self.create_publisher(Float32, 'humidity_data', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Publish every 2 seconds

        try:
            self.bus = smbus2.SMBus(I2C_BUS)
            self.get_logger().info("I2C bus initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C bus: {e}")
            self.bus = None

    def read_sht30(self):
        if self.bus is None:
            return None, None

        try:
            # Send measurement command
            self.bus.write_i2c_block_data(SHT30_ADDRESS, MEASUREMENT_COMMAND[0], [MEASUREMENT_COMMAND[1]])
            time.sleep(0.02)  # Wait for measurement

            # Read 6 bytes of data (Status, Temperature MSB, Temperature LSB, CRC, Humidity MSB, Humidity LSB, CRC)
            data = self.bus.read_i2c_block_data(SHT30_ADDRESS, 0x00, 6)

            # Calculate temperature
            temp_msb, temp_lsb, temp_crc = data[0], data[1], data[2]
            temperature = -45 + (175 * ((temp_msb * 256 + temp_lsb) / 65535.0))

            # Calculate humidity
            humidity_msb, humidity_lsb, humidity_crc = data[3], data[4], data[5]
            humidity = 100 * ((humidity_msb * 256 + humidity_lsb) / 65535.0)

            return temperature, humidity
        except Exception as e:
            self.get_logger().error(f"Error reading sensor data: {e}")
            return None, None

    def timer_callback(self):
        temperature, humidity = self.read_sht30()
        if temperature is not None and humidity is not None:
            self.temp_publisher.publish(Float32(data=temperature))
            self.humidity_publisher.publish(Float32(data=humidity))
            self.get_logger().info(f"Published Temperature: {temperature:.2f} °C, Humidity: {humidity:.2f} %")
        else:
            self.get_logger().warn("Failed to read sensor data.")

def main(args=None):
    rclpy.init(args=args)
    node = TempHumidityPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()