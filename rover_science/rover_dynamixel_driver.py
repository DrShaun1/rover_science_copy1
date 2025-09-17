#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from dynamixel_sdk import *
import time

# Dynamixel SDK constants
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

        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.initialize_connection()

        self.motors = {}  # motor_id -> config

    def initialize_connection(self):
        if not self.portHandler.openPort():
            self.get_logger().fatal("Failed to open port.")
            raise RuntimeError("Failed to open port.")
        self.get_logger().info("Port opened successfully.")

        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().fatal("Failed to set baudrate.")
            raise RuntimeError("Failed to set baudrate.")
        self.get_logger().info(f"Baudrate set to {BAUDRATE}.")

    def add_motor(self, motor_id, mode, topic, min_position=0, max_position=4095, max_velocity=1023):
        self.motors[motor_id] = {
            'mode': mode,
            'topic': topic,
            'min_position': min_position,
            'max_position': max_position,
            'max_velocity': max_velocity,
        }

        if mode == 'position':
            self.configure_motor(motor_id, POSITION_CONTROL, "Position Control")
        elif mode == 'velocity':
            self.configure_motor(motor_id, VELOCITY_CONTROL, "Velocity Control")
        else:
            self.get_logger().error(f"Unknown mode '{mode}' for motor ID {motor_id}")
            return

        self.create_subscription(Float32, topic, lambda msg, m_id=motor_id: self.motor_callback(m_id, msg), 10)
        self.get_logger().info(f"Subscribed to topic '{topic}' for motor ID {motor_id}")

    def configure_motor(self, motor_id, operating_mode, mode_name):
        try:
            self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_OPERATING_MODE, operating_mode)
            self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, 1)

            # Verify torque enable
            torque_enabled, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE)
            if torque_enabled != 1:
                self.get_logger().error(f"Failed to enable torque for motor {motor_id}.")
            else:
                self.get_logger().info(f"Motor {motor_id} configured for {mode_name}.")
        except Exception as e:
            self.get_logger().warn(f"Could not configure motor {motor_id}: {e}")

    def motor_callback(self, motor_id, msg):
        try:
            value = msg.data
            if motor_id not in self.motors:
                self.get_logger().error(f"Motor ID {motor_id} not found in configuration.")
                return

            config = self.motors[motor_id]

            # Check if the value is -1.0, indicating a reboot command
            if abs(value - (-1.0)) < 1e-6:
                self.get_logger().info(f"Received reboot command for motor ID {motor_id}")
                self.reboot_motor(motor_id)
                return

            # Handle position or velocity control
            if config['mode'] == 'position':
                clamped = max(config['min_position'], min(config['max_position'], value))
                self.set_position(motor_id, clamped)
            elif config['mode'] == 'velocity':
                clamped = max(-config['max_velocity'], min(config['max_velocity'], value))
                self.set_velocity(motor_id, clamped)
            else:
                self.get_logger().error(f"Unknown mode '{config['mode']}' for motor ID {motor_id}")
        except Exception as e:
            self.get_logger().error(f"Error in motor_callback for motor ID {motor_id}: {e}")

    def set_position(self, motor_id, position):
        self.get_logger().info(f"Motor {motor_id} → Position: {position}")
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor_id, ADDR_GOAL_POSITION, int(position))
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Comm error on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Dynamixel error on motor {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")

    def set_velocity(self, motor_id, velocity):
        self.get_logger().info(f"Motor {motor_id} → Velocity: {velocity}")
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor_id, ADDR_GOAL_VELOCITY, int(velocity))
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Comm error on motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Dynamixel error on motor {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")

    def reboot_motor(self, motor_id):
        self.get_logger().info(f"Rebooting motor ID {motor_id}...")
        try:
            dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, motor_id)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"Failed to reboot motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return False
            elif dxl_error != 0:
                self.get_logger().error(f"Motor {motor_id} reboot error: {self.packetHandler.getRxPacketError(dxl_error)}")
                return False

            self.get_logger().info(f"Motor {motor_id} rebooted successfully.")

            # Allow time for the motor to stabilize
            time.sleep(0.5)

            # Reconfigure the motor after reboot
            if motor_id in self.motors:
                config = self.motors[motor_id]
                self.configure_motor(motor_id, POSITION_CONTROL if config['mode'] == 'position' else VELOCITY_CONTROL, config['mode'].capitalize())

                # Explicitly set the motor to its current position or a default position
                current_position = config['min_position']  # Default to min_position
                self.set_position(motor_id, current_position)
                self.get_logger().info(f"Motor {motor_id} initialized to position {current_position} after reboot.")
            else:
                self.get_logger().warn(f"Motor ID {motor_id} not found in configuration. Skipping reconfiguration.")

            return True
        except Exception as e:
            self.get_logger().error(f"Exception during reboot of motor {motor_id}: {e}")
            return False

    def destroy_node(self):
        self.portHandler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelController()

    # Add motors here
    node.add_motor(motor_id=4, mode='position', topic='Purge2', min_position=1470, max_position=2050)
    node.add_motor(motor_id=7, mode='position', topic='Splitter', min_position=0, max_position=320)
    node.add_motor(motor_id=3, mode='position', topic='Purge1', min_position=1000, max_position=1565)
    node.add_motor(motor_id=5, mode='position', topic='SampleCache', min_position=0, max_position=2000)
    node.add_motor(motor_id=1, mode='position', topic='Sample1', min_position=100, max_position=3000)

    # Reboot example (optional - can be triggered based on logic)
    # node.reboot_motor(4)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
