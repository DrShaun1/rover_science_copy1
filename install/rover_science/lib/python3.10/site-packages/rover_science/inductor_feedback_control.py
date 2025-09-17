#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class NichromeTempController(Node):
    def __init__(self):
        super().__init__('nichrome_temp_controller')
        # Subscribe to the SCD-41 temperature topic
        self.create_subscription(
            Float32, 
            'scd41_temperature_data', 
            self.temp_callback, 
            10
        )
        # Subscribe to the original nichrome command topic (from the VESC driver)
        self.create_subscription(
            Float32, 
            'nichrome', 
            self.nichrome_callback, 
            10
        )
        # Publisher to override/update the nichrome current command
        self.nichrome_publisher = self.create_publisher(Float32, 'nichrome', 10)
        
        # Timer for periodic control (adjust frequency as needed)
        self.create_timer(0.1, self.control_loop)
        
        # Store the latest received values
        self.last_temp = None
        self.last_nichrome_command = None
        
        # Control parameters (in °C)
        self.target_temp = 21.5  # Target temperature in °C
        self.max_current = 19.5  # Maximum current in A

    def temp_callback(self, msg: Float32):
        self.last_temp = msg.data
        self.get_logger().info(f"Received temperature: {self.last_temp} °C")
        print(f"Received temperature: {self.last_temp} °C")

    def nichrome_callback(self, msg: Float32):
        self.last_nichrome_command = msg.data
        self.get_logger().info(f"Received baseline nichrome command: {self.last_nichrome_command}")
        print(f"Received baseline nichrome command: {self.last_nichrome_command}")

    def control_loop(self):
        # Make sure we have received both messages before acting
        if self.last_temp is None or self.last_nichrome_command is None:
            return

        # Ensure the last nichrome command is not below 0A
        if self.last_nichrome_command < 0:
            self.last_nichrome_command = 0.0

        # Calculate the adjustment factor based on the temperature
        if self.last_temp > self.target_temp:
            # Decrease current exponentially if temperature is above target
            adjustment_factor = math.exp(-(self.last_temp - self.target_temp))
        else:
            # Increase current exponentially if temperature is below target
            adjustment_factor = math.exp(self.target_temp - self.last_temp)
        
        adjusted_command = self.last_nichrome_command * adjustment_factor

        # Ensure the adjusted command does not exceed the maximum current
        if adjusted_command > self.max_current:
            adjusted_command = self.max_current

        # Ensure the adjusted command is not below 1mA (0.001A)
        if adjusted_command < 0.001:
            adjusted_command = 0.001

        # Publish the adjusted nichrome command
        out_msg = Float32()
        out_msg.data = adjusted_command
        self.nichrome_publisher.publish(out_msg)
        self.get_logger().info(
            f"Adjusted nichrome command: {adjusted_command} (adjustment factor: {adjustment_factor:.2f})"
        )
        print(f"Adjusted nichrome command: {adjusted_command} (adjustment factor: {adjustment_factor:.2f})")
        self.get_logger().info(f"Current output: {adjusted_command}")
        print(f"Current output: {adjusted_command}")

def main(args=None):
    rclpy.init(args=args)
    node = NichromeTempController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()