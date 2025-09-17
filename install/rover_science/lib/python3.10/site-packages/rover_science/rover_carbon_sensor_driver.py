import time
import board
import adafruit_ads1x15.ads1115 as ADS
import adafruit_scd4x
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class CarbonSensorPublisher(Node):
    def __init__(self):
        super().__init__('carbon_sensor_publisher')
        self.co2_publisher = self.create_publisher(Float32, 'co2_data', 10)
        self.temp_publisher = self.create_publisher(Float32, 'scd41_temperature_data', 10)
        self.humidity_publisher = self.create_publisher(Float32, 'scd41_humidity_data', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)  # Timer calls `timer_callback` every 5 seconds

        try:
            i2c = board.I2C()
            self.scd4x = adafruit_scd4x.SCD4X(i2c)
            self.scd4x.start_periodic_measurement()
            self.get_logger().info("Waiting for first measurement....")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensor: {e}")
            return

    def timer_callback(self):
        """Callback function to periodically publish sensor data."""
        try:
            if self.scd4x.data_ready:
                co2 = float(self.scd4x.CO2)
                temperature = float(self.scd4x.temperature)
                humidity = float(self.scd4x.relative_humidity)

                self.co2_publisher.publish(Float32(data=co2))
                self.temp_publisher.publish(Float32(data=temperature))
                self.humidity_publisher.publish(Float32(data=humidity))

                self.get_logger().info(f"Publishing CO2: {co2} ppm")
                self.get_logger().info(f"Publishing Temperature: {temperature} °C")
                self.get_logger().info(f"Publishing Humidity: {humidity} %")
            else:
                self.get_logger().warn("Sensor data not ready.")
        except Exception as e:
            self.get_logger().error(f"Error reading sensor data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CarbonSensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()