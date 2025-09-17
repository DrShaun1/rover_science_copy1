import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import LED


class GPIORosController(Node):
    def __init__(self):
        super().__init__('gpio_ros_controller')

        # Initialize GPIO pins
        self.UV = LED(16)  # GPIO16
        self.VIBRATION = LED(12)  # GPIO12

        self.get_logger().info("GPIO pins initialized: UV=GPIO16, VIBRATION=GPIO12")

        # Subscribe to ROS2 topics
        self.uv_subscription = self.create_subscription(
            Bool,
            'uv_toggle',
            self.uv_callback,
            10
        )
        self.get_logger().info("Subscribed to 'uv_toggle' topic")

        self.vibration_subscription = self.create_subscription(
            Bool,
            'vibration_toggle',
            self.vibration_callback,
            10
        )
        self.get_logger().info("Subscribed to 'vibration_toggle' topic")

    def uv_callback(self, msg):
        """Callback to handle UV GPIO control."""
        if msg.data:
            self.UV.on()  # Turn UV GPIO pin ON
            self.get_logger().info("UV set to HIGH")
        else:
            self.UV.off()  # Turn UV GPIO pin OFF
            self.get_logger().info("UV set to LOW")

    def vibration_callback(self, msg):
        """Callback to handle VIBRATION GPIO control."""
        if msg.data:
            self.VIBRATION.on()  # Turn VIBRATION GPIO pin ON
            self.get_logger().info("VIBRATION set to HIGH")
        else:
            self.VIBRATION.off()  # Turn VIBRATION GPIO pin OFF
            self.get_logger().info("VIBRATION set to LOW")


def main(args=None):
    rclpy.init(args=args)
    node = GPIORosController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup GPIO pins before shutting down
        node.UV.off()
        node.VIBRATION.off()
        node.get_logger().info("GPIO pins reset to LOW")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()