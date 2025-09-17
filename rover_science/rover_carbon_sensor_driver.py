import time
import board
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
        self.timer = self.create_timer(5.0, self.timer_callback)  # every 5 seconds
        self.sensor_ready_time = None

        try:
            i2c = board.I2C()
            self.scd4x = adafruit_scd4x.SCD4X(i2c)
            
            # Stop any running measurements to configure
            self.scd4x.stop_periodic_measurement()
            time.sleep(1)

            # Sensor optimization
            self.scd4x.sensor_altitude = 150
            self.get_logger().info(f"Sensor altitude set to {self.scd4x.sensor_altitude} meters")

            self.scd4x.auto_self_calibration_enabled = True
            self.get_logger().info(f"ASC enabled: {self.scd4x.auto_self_calibration_enabled}")

            # Log serial number
            serial = self.scd4x.serial_number
            serial_str = ''.join(f'{part:04X}' for part in serial)
            self.get_logger().info(f"SCD-41 serial number: {serial_str}")


            # Start measurement
            self.scd4x.start_periodic_measurement()
            self.sensor_ready_time = time.time() + 60  # wait ~1 min before expecting good CO2

            self.get_logger().info("SCD-41 started. Warming up for 60 seconds...")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensor: {e}")
            self.scd4x = None

    def timer_callback(self):
        if self.scd4x is None:
            return

        try:
            if self.scd4x.data_ready:
                co2 = float(self.scd4x.CO2)
                temperature = float(self.scd4x.temperature)
                humidity = float(self.scd4x.relative_humidity)

                # Wait until sensor warm-up is complete
                if time.time() < self.sensor_ready_time:
                    self.get_logger().info("Warming up... ignoring early CO2 readings.")
                else:
                    if co2 <= 0.0:
                        self.get_logger().warn("CO2 reading is 0.0 ppm — check sensor exposure to air.")
                    else:
                        self.co2_publisher.publish(Float32(data=co2))
                        self.get_logger().info(f"CO2: {co2} ppm")

                self.temp_publisher.publish(Float32(data=temperature))
                self.humidity_publisher.publish(Float32(data=humidity))
                self.get_logger().info(f"Temp: {temperature:.2f} °C | RH: {humidity:.2f} %")
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
