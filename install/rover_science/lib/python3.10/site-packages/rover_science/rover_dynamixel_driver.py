#!/usr/bin/env python3
"""
This module provides a ROS2 node for dynamically controlling Dynamixel motors using the Dynamixel SDK.
The node allows users to define motors dynamically with their respective modes (velocity or position).
"""

import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
from std_msgs.msg import Float32
from std_srvs.srv import Trigger  # Import Trigger service type

# Constants
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICE_NAME = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9MIR8R-if00-port0'

# Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_POSITION = 116
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128

# Operating Modes
POSITION_CONTROL = 3
VELOCITY_CONTROL = 1


class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')

        # Create a PortHandler instance for the specified device
        self.portHandler = PortHandler(DEVICE_NAME)
        # Create a PacketHandler instance for the specified protocol version
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize the connection to the motors
        self.initialize_connection()

        # Dictionary to store motor configurations
        self.motors = {}

    def initialize_connection(self):
        """Initializes the connection to the Dynamixel motors."""
        if not self.portHandler.openPort():
            self.get_logger().fatal("Port open failed! Check if U2D2 is connected.")
            raise RuntimeError("Failed to open port.")
        self.get_logger().info("Port opened successfully.")

        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().fatal("Baudrate set failed!")
            raise RuntimeError("Failed to set baudrate.")
        self.get_logger().info(f"Connection initialized. Baudrate: {BAUDRATE}")

    def add_motor(self, motor_id, mode, topic, min_position=0, max_position=4095, max_velocity=1023):
        """
        Adds a motor dynamically to the controller.

        :param motor_id: The ID of the motor.
        :param mode: The operating mode ('position' or 'velocity').
        :param topic: The ROS2 topic to subscribe to for controlling the motor.
        :param min_position: The minimum position limit for position control.
        :param max_position: The maximum position limit for position control.
        :param max_velocity: The maximum velocity for velocity control.
        """
        self.motors[motor_id] = {
            'mode': mode,
            'topic': topic,
            'min_position': min_position,
            'max_position': max_position,
            'max_velocity': max_velocity,
        }

        # Configure the motor based on the mode
        if mode == 'position':
            self.configure_motor(motor_id, POSITION_CONTROL, "Position Control")
        elif mode == 'velocity':
            self.configure_motor(motor_id, VELOCITY_CONTROL, "Velocity Control")
        else:
            self.get_logger().error(f"Unknown mode '{mode}' for motor ID {motor_id}")
            return

        # Create a subscription for the motor
        self.create_subscription(Float32, topic, lambda msg, motor_id=motor_id: self.motor_callback(motor_id, msg), 10)
        self.get_logger().info(f"Subscription created for motor ID {motor_id} on topic '{topic}'")

    def configure_motor(self, motor_id, mode, mode_name):
        """Configures a motor with the specified operating mode."""
        self.get_logger().info(f"Configuring Motor ID {motor_id} for {mode_name}...")
        try:
            # Disable torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, 0)
            if dxl_comm_result != COMM_SUCCESS:
                raise RuntimeError(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                raise RuntimeError(self.packetHandler.getRxPacketError(dxl_error))

            # Set operating mode
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_OPERATING_MODE, mode)
            if dxl_comm_result != COMM_SUCCESS:
                raise RuntimeError(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                raise RuntimeError(self.packetHandler.getRxPacketError(dxl_error))

            # Enable torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, 1)
            if dxl_comm_result != COMM_SUCCESS:
                raise RuntimeError(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                raise RuntimeError(self.packetHandler.getRxPacketError(dxl_error))

            self.get_logger().info(f"Motor ID {motor_id} configured successfully for {mode_name}.")
        except RuntimeError as e:
            self.get_logger().warning(f"Motor ID {motor_id} is not detected: {e}")

    def motor_callback(self, motor_id, msg):
        """Callback to handle incoming ROS2 messages and control the motor."""
        value = msg.data
        motor_config = self.motors[motor_id]
        mode = motor_config['mode']

        if mode == 'position':
            # Clamp the position value to the min and max position limits
            min_position = motor_config['min_position']
            max_position = motor_config['max_position']
            clamped_position = max(min_position, min(value, max_position))
            self.set_position(motor_id, clamped_position)
        elif mode == 'velocity':
            # Scale the velocity value based on max_velocity
            max_velocity = motor_config['max_velocity']
            scaled_velocity = max(-max_velocity, min(value, max_velocity))
            self.set_velocity(motor_id, scaled_velocity)
        else:
            self.get_logger().error(f"Unknown mode '{mode}' for motor ID {motor_id}")

    def set_position(self, motor_id, position):
        """Sets the position of the specified motor."""
        self.get_logger().info(f"Setting Position for Motor ID {motor_id}: {position}")
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_GOAL_POSITION, int(position))
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Motor ID {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Motor ID {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")

    def set_velocity(self, motor_id, velocity):
        """Sets the velocity of the specified motor."""
        self.get_logger().info(f"Setting Velocity for Motor ID {motor_id}: {velocity}")
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_GOAL_VELOCITY, int(velocity))
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Motor ID {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Motor ID {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")

    def reboot_motor(self, motor_id):
        """Reboots the specified motor to recover from errors."""
        self.get_logger().info(f"Attempting to reboot Motor ID {motor_id}...")
        try:
            # Reboot the motor
            dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, motor_id)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"Reboot failed for Motor ID {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"Reboot error for Motor ID {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"Motor ID {motor_id} rebooted successfully.")
        except Exception as e:
            self.get_logger().error(f"Exception during reboot of Motor ID {motor_id}: {e}")

    def create_reboot_service(self):
        """Creates a ROS2 service to reboot motors."""
        self.reboot_service = self.create_service(
            srv_type=Trigger,  # Use the Trigger service type
            srv_name='reboot_motor',
            callback=self.handle_reboot_request
        )
        self.get_logger().info("Reboot service created.")

    def handle_reboot_request(self, request, response):
        """Handles incoming reboot requests."""
        try:
            motor_id = int(request.motor_id)  # Ensure motor_id is an integer
            self.reboot_motor(motor_id)
            response.success = True
            response.message = f"Motor ID {motor_id} rebooted successfully."
        except Exception as e:
            response.success = False
            response.message = f"Failed to reboot motor: {e}"
        return response

    def destroy_node(self):
        """Clean up resources when shutting down."""
        self.portHandler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelController()

    # Add motors dynamically
    node.add_motor(motor_id=4, mode='position', topic='Purge2', min_position=1536, max_position=2002)
    node.add_motor(motor_id=7, mode='position', topic='Splitter', min_position=0, max_position=320)
    node.add_motor(motor_id=3, mode='position', topic='Purge1', min_position=0, max_position=500)
    node.add_motor(motor_id=5, mode='position', topic='SampleCache', min_position=495, max_position=2582)
    node.add_motor(motor_id=1, mode='position', topic='Sample1', min_position=100, max_position=3000)

    # Create reboot service
    node.create_reboot_service()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Check if shutdown has not already been called
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()