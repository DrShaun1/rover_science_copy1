import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import re

class ArduinoPublisher(Node):
    def __init__(self):
        super().__init__('arduino_publisher')
        # Create publishers for temperature and soil moisture
        self.temp1_publisher = self.create_publisher(Float32, 'thermocouple1_data', 10)
        self.temp2_publisher = self.create_publisher(Float32, 'thermocouple2_data', 10)
        self.soil_moisture_publisher = self.create_publisher(Float32, 'soil_moisture_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        try:
            # Open serial connection to Arduino
            self.ser = serial.Serial('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', 9600, timeout=1)
            self.ser.flush()
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to the serial port: {e}")
            rclpy.shutdown()

    def timer_callback(self):
        # Ensure data is available to read
        if self.ser.in_waiting > 0:
            try:
                # Read the line from serial and decode it
                line = self.ser.readline().decode('utf-8').rstrip()
                
                # Log the raw line for debugging (optional)
                self.get_logger().info(f"Raw line from Arduino: {line}")

                # Regular expressions to match the temperature and soil moisture
                temp1_match = re.search(r'Thermocouple 1: (\d*\.?\d+)', line)
                temp2_match = re.search(r'Thermocouple 2: (\d*\.?\d+)', line)
                moisture_match = re.search(r'Soil Moisture Value: (\d+)', line)

                # Publish the temperature and soil moisture if found
                if temp1_match:
                    temp1 = float(temp1_match.group(1))
                    self.temp1_publisher.publish(Float32(data=temp1))
                    self.get_logger().info(f"Publishing Thermocouple 1: {temp1} °C")

                if temp2_match:
                    temp2 = float(temp2_match.group(1))
                    self.temp2_publisher.publish(Float32(data=temp2))
                    self.get_logger().info(f"Publishing Thermocouple 2: {temp2} °C")

                if moisture_match:
                    moisture = float(moisture_match.group(1))
                    self.soil_moisture_publisher.publish(Float32(data=moisture))
                    self.get_logger().info(f"Publishing Soil Moisture Value: {moisture}")

            except Exception as e:
                self.get_logger().error(f"Error reading from serial: {e}")
        else:
            self.get_logger().info("No data available to read from serial.")

def main(args=None):
    rclpy.init(args=args)
    arduino_publisher = ArduinoPublisher()
    try:
        rclpy.spin(arduino_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        arduino_publisher.ser.close()
        arduino_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()