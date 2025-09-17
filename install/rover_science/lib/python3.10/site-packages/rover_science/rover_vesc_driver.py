import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pyvesc import VESC
import os


class VESCController(Node):
    def __init__(self, serial_port=None):
        super().__init__('vesc_controller')

        # Use the provided serial port or a default value
        self.serial_port = serial_port or os.getenv('VESC_SERIAL_PORT', '/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00')
        self.motor = VESC(serial_port=self.serial_port)

        # Dictionary to store motor configurations
        self.motors = {}

    def add_motor(self, motor_id, control_type, topic, max_value=1.0):
        """
        Add a motor to the controller.

        :param motor_id: The ID of the motor (0 for UART-connected VESC).
        :param control_type: The control type ('current' or 'duty_cycle').
        :param topic: The ROS2 topic to subscribe to for controlling the motor.
        :param max_value: The maximum value for the control type.
        """
        self.motors[motor_id] = {
            'control_type': control_type,
            'topic': topic,
            'max_value': max_value,
        }

        # Skip starting the heartbeat for motor ID 0 (UART connection)
        if motor_id != 0:
            try:
                # Start heartbeat for the motor
                self.motor.start_heartbeat(can_id=motor_id)
                self.get_logger().info(f"Heartbeat started for motor ID {motor_id}")
            except Exception as e:
                self.get_logger().error(f"Failed to start heartbeat for motor ID {motor_id}: {e}")
        else:
            self.get_logger().info(f"Skipping heartbeat for motor ID {motor_id} (UART connection)")

        # Create a subscription for the motor
        self.create_subscription(Float32, topic, lambda msg, motor_id=motor_id: self.motor_callback(motor_id, msg), 10)
        self.get_logger().info(f"Subscription created for motor ID {motor_id} on topic '{topic}'")

    def motor_callback(self, motor_id, msg):
        """Callback to handle incoming ROS2 messages and send commands to the VESC."""
        value = msg.data  # Use the raw value directly
        motor_config = self.motors[motor_id]
        control_type = motor_config['control_type']

        try:
            if control_type == 'current':
                if motor_id == 0:  # UART-connected VESC
                    self.motor.set_current(value)
                else:
                    self.motor.set_current(value, can_id=motor_id)
                self.get_logger().info(f"Set current for motor ID {motor_id}: {value}")
            elif control_type == 'duty_cycle':
                if motor_id == 0:  # UART-connected VESC
                    self.motor.set_duty_cycle(value)
                else:
                    self.motor.set_duty_cycle(value, can_id=motor_id)
                self.get_logger().info(f"Set duty cycle for motor ID {motor_id}: {value}")
            else:
                self.get_logger().error(f"Unknown control type '{control_type}' for motor ID {motor_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command to motor ID {motor_id}: {e}")

    def destroy_node(self):
        """Clean up resources when shutting down."""
        self.motor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VESCController()

    # Add motors dynamically
    node.add_motor(motor_id=111, control_type='current', topic='pump', max_value=0.8)
    node.add_motor(motor_id=21, control_type='duty_cycle', topic='dmotor1', max_value=0.7)
    node.add_motor(motor_id=0, control_type='duty_cycle', topic='dmotor2', max_value=0.7)  # UART-connected VESC
    node.add_motor(motor_id=44, control_type='duty_cycle', topic='inductor', max_value=1)
    node.add_motor(motor_id=47, control_type='duty_cycle', topic='linear1', max_value=0.10)
    node.add_motor(motor_id=64, control_type='duty_cycle', topic='linear2', max_value=0.10)
    node.add_motor(motor_id=81, control_type='duty_cycle', topic='mixer', max_value=1.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()