import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import csv
import os
import time

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        # Subscribe to topics from TempHumidityPublisher and CarbonSensorPublisher
        self.create_subscription(Float32, 'temperature_data', self.temperature_callback, 10)
        self.create_subscription(Float32, 'humidity_data', self.humidity_callback, 10)
        self.create_subscription(Float32, 'co2_data', self.co2_callback, 10)
        self.create_subscription(Float32, 'scd41_temperature_data', self.scd41_temperature_callback, 10)

        # Initialize start time for logging
        self.start_time = time.time()

        # Create folder structure for logging
        self.data_path = os.path.expanduser('~/sensor-data')
        os.makedirs(self.data_path, exist_ok=True)

        # Open CSV files for logging
        self.temp_csv = open(os.path.join(self.data_path, 'temperature_data.csv'), 'a', newline='')
        self.humidity_csv = open(os.path.join(self.data_path, 'humidity_data.csv'), 'a', newline='')
        self.co2_csv = open(os.path.join(self.data_path, 'co2_data.csv'), 'a', newline='')

        # Create CSV writers
        self.temp_writer = csv.writer(self.temp_csv)
        self.humidity_writer = csv.writer(self.humidity_csv)
        self.co2_writer = csv.writer(self.co2_csv)

        # Write headers if the files are empty
        if os.stat(self.temp_csv.name).st_size == 0:
            self.temp_writer.writerow(['Time (s)', 'Temperature (°C)'])
        if os.stat(self.humidity_csv.name).st_size == 0:
            self.humidity_writer.writerow(['Time (s)', 'Humidity (%)'])
        if os.stat(self.co2_csv.name).st_size == 0:
            self.co2_writer.writerow(['Time (s)', 'CO2 (ppm)'])

    def temperature_callback(self, msg):
        current_time = time.time() - self.start_time
        temperature = round(msg.data, 2)
        self.temp_writer.writerow([current_time, temperature])
        self.temp_csv.flush()  # Ensure data is written to disk
        self.get_logger().info(f"Logged Temperature: {temperature}°C")

    def humidity_callback(self, msg):
        current_time = time.time() - self.start_time
        humidity = round(msg.data, 2)
        self.humidity_writer.writerow([current_time, humidity])
        self.humidity_csv.flush()  # Ensure data is written to disk
        self.get_logger().info(f"Logged Humidity: {humidity}%")

    def co2_callback(self, msg):
        current_time = time.time() - self.start_time
        co2 = round(msg.data, 2)
        self.co2_writer.writerow([current_time, co2])
        self.co2_csv.flush()  # Ensure data is written to disk
        self.get_logger().info(f"Logged CO2: {co2} ppm")

    def scd41_temperature_callback(self, msg):
        current_time = time.time() - self.start_time
        temperature = round(msg.data, 2)
        self.temp_writer.writerow([current_time, temperature])
        self.temp_csv.flush()  # Ensure data is written to disk
        self.get_logger().info(f"Logged SCD41 Temperature: {temperature}°C")

    def destroy_node(self):
        # Close CSV files when shutting down
        if not self.temp_csv.closed:
            self.temp_csv.close()
        if not self.humidity_csv.closed:
            self.humidity_csv.close()
        if not self.co2_csv.closed:
            self.co2_csv.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()