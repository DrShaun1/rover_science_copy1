import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import tkinter as tk
from tkinter import ttk


class ROS2GUINode(Node):
    def __init__(self):
        super().__init__('ros2_gui')

        # VESC motor settings
        self.vesc_settings = {
            'pump': {'current': 0.8, 'publisher': self.create_publisher(Float32, 'pump', 10)},
            'dmotor1': {'duty_cycle': 0.7, 'publisher': self.create_publisher(Float32, 'dmotor1', 10)},
            'dmotor2': {'duty_cycle': 0.7, 'publisher': self.create_publisher(Float32, 'dmotor2', 10)},
            'inductor': {'duty_cycle': 1.0, 'publisher': self.create_publisher(Float32, 'inductor', 10)},
            'linear1': {'duty_cycle': 0.25, 'publisher': self.create_publisher(Float32, 'linear1', 10)},
            'linear2': {'duty_cycle': 0.25, 'publisher': self.create_publisher(Float32, 'linear2', 10)},
            'mixer': {'duty_cycle': 1.0, 'publisher': self.create_publisher(Float32, 'mixer', 10)},
        }

        # Dynamixel motor settings
        self.dynamixel_settings = {
            'Purge2': {'min_position': 1470, 'max_position': 2040, 'motor_id': 4, 'min_label': 'Close', 'max_label': 'Open', 'topic': 'Purge2'},
            'Splitter': {'min_position': 0, 'max_position': 320, 'motor_id': 7, 'min_label': 'Cache', 'max_label': 'Inductor', 'topic': 'Splitter'},
            'Purge1': {'min_position': 1000, 'max_position': 1565, 'motor_id': 3, 'min_label': 'Open', 'max_label': 'Close', 'topic': 'Purge1'},
            'SampleCache': {'min_position': 0, 'max_position': 2000, 'motor_id': 5, 'min_label': 'Close', 'max_label': 'Open', 'topic': 'SampleCache'},
            'Sample1': {'min_position': 100, 'max_position': 3000, 'motor_id': 1, 'min_label': 'Retract', 'max_label': 'Extend', 'topic': 'Sample1'},
        }

        self.dynamixel_publishers = {
            settings['topic']: self.create_publisher(Float32, settings['topic'], 10)
            for key, settings in self.dynamixel_settings.items()
        }

        # Publishers for UV and Vibration
        self.uv_publisher = self.create_publisher(Bool, 'uv_toggle', 10)
        self.vibration_publisher = self.create_publisher(Bool, 'vibration_toggle', 10)

        # Initialize the GUI
        self.root = tk.Tk()
        self.root.title("ROS2 Motor Control GUI")

        # Create frames for VESC and Dynamixel motors
        vesc_frame = tk.LabelFrame(self.root, text="VESC Motors", padx=10, pady=10)
        vesc_frame.grid(row=0, column=0, padx=10, pady=10, sticky="n")

        # Add a canvas and scrollbar to the VESC frame
        canvas = tk.Canvas(vesc_frame)
        scrollbar = ttk.Scrollbar(vesc_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Add VESC motor controls to the scrollable frame
        for key, settings in self.vesc_settings.items():
            frame = tk.Frame(scrollable_frame)
            frame.pack(pady=5)

            tk.Label(frame, text=key).pack()

            if key == 'pump':  # Pump motor with ON/OFF/REVERSE buttons
                on_button = tk.Button(frame, text="ON", command=lambda k=key: self.update_vesc_value(k, 0.8))
                off_button = tk.Button(frame, text="OFF", command=lambda k=key: self.update_vesc_value(k, 0.0))
                reverse_button = tk.Button(frame, text="REVERSE", command=lambda k=key: self.update_vesc_value(k, -0.8))
                on_button.pack(side=tk.LEFT, padx=5)
                off_button.pack(side=tk.LEFT, padx=5)
                reverse_button.pack(side=tk.LEFT, padx=5)

            elif key in ['linear1', 'linear2']:  # Linear motors with UP, 0, DOWN buttons
                up_button = tk.Button(frame, text="DOWN (0.05)", command=lambda k=key: self.update_vesc_value(k, 0.05))
                zero_button = tk.Button(frame, text="0", command=lambda k=key: self.update_vesc_value(k, 0.0))
                down_button = tk.Button(frame, text="UP (-0.1)", command=lambda k=key: self.update_vesc_value(k, -0.1))
                up_button.pack(side=tk.LEFT, padx=5)
                zero_button.pack(side=tk.LEFT, padx=5)
                down_button.pack(side=tk.LEFT, padx=5)

            elif key in ['dmotor1', 'dmotor2']:  # Drill motors with ON (-0.33) and OFF (0) buttons
                on_button = tk.Button(frame, text="ON (-0.33)", command=lambda k=key: self.update_vesc_value(k, -0.33))
                off_button = tk.Button(frame, text="OFF (0)", command=lambda k=key: self.update_vesc_value(k, 0.0))
                on_button.pack(side=tk.LEFT, padx=5)
                off_button.pack(side=tk.LEFT, padx=5)

            elif key == 'mixer':  # Mixer motor with ON (0.20) and OFF (0) buttons
                on_button = tk.Button(frame, text="ON (0.20)", command=lambda k=key: self.update_vesc_value(k, 0.20))
                off_button = tk.Button(frame, text="OFF (0)", command=lambda k=key: self.update_vesc_value(k, 0.0))
                on_button.pack(side=tk.LEFT, padx=5)
                off_button.pack(side=tk.LEFT, padx=5)

            elif key == 'inductor':  # Inductor motor with ON (1.0) and OFF (0) buttons
                on_button = tk.Button(frame, text="ON (1.0)", command=lambda k=key: self.update_vesc_value(k, 1.0))
                off_button = tk.Button(frame, text="OFF (0)", command=lambda k=key: self.update_vesc_value(k, 0.0))
                on_button.pack(side=tk.LEFT, padx=5)
                off_button.pack(side=tk.LEFT, padx=5)

        # Add a button to set all VESC motor values to zero
        reset_all_button = tk.Button(scrollable_frame, text="Set All to Zero", command=self.set_all_vesc_to_zero)
        reset_all_button.pack(pady=10)

        dynamixel_frame = tk.LabelFrame(self.root, text="Dynamixel Motors", padx=10, pady=10)
        dynamixel_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")

        # Create a frame for GPIO controls in the third column
        gpio_frame = tk.LabelFrame(self.root, text="GPIO Controls", padx=10, pady=10)
        gpio_frame.grid(row=0, column=2, padx=10, pady=10, sticky="n")

        # Add GPIO controls for UV
        tk.Label(gpio_frame, text="UV Control").pack()
        uv_on_button = tk.Button(gpio_frame, text="Turn UV ON", command=lambda: self.toggle_uv(True))
        uv_on_button.pack(pady=5)
        uv_off_button = tk.Button(gpio_frame, text="Turn UV OFF", command=lambda: self.toggle_uv(False))
        uv_off_button.pack(pady=5)

        # Add GPIO controls for Vibration
        tk.Label(gpio_frame, text="Vibration Control").pack()
        vibration_on_button = tk.Button(gpio_frame, text="Turn Vibration ON", command=lambda: self.toggle_vibration(True))
        vibration_on_button.pack(pady=5)
        vibration_off_button = tk.Button(gpio_frame, text="Turn Vibration OFF", command=lambda: self.toggle_vibration(False))
        vibration_off_button.pack(pady=5)

        # Add Dynamixel motor controls
        for key, settings in self.dynamixel_settings.items():
            frame = tk.Frame(dynamixel_frame)
            frame.pack(pady=5)

            tk.Label(frame, text=key).pack()

            min_position = settings['min_position']
            max_position = settings['max_position']
            motor_id = settings['motor_id']
            min_label = settings['min_label']
            max_label = settings['max_label']

            # Add a button to move to the minimum position
            min_button = tk.Button(frame, text=min_label,
                                    command=lambda m_id=motor_id, pos=min_position: self.move_dynamixel_motor(m_id, pos))
            min_button.pack(pady=5)

            # Add a button to move to the maximum position
            max_button = tk.Button(frame, text=max_label,
                                    command=lambda m_id=motor_id, pos=max_position: self.move_dynamixel_motor(m_id, pos))
            max_button.pack(pady=5)

            # Add a button to reboot the motor
            reboot_button = tk.Button(frame, text="Reboot Motor",
                                       command=lambda m_id=motor_id: self.reboot_dynamixel_motor(m_id))
            reboot_button.pack(pady=5)

        # Start the Tkinter loop
        self.root.mainloop()

    def toggle_uv(self, state):
        """Publishes a message to toggle the UV GPIO."""
        msg = Bool()
        msg.data = state
        self.uv_publisher.publish(msg)
        self.get_logger().info(f"UV set to {'ON' if state else 'OFF'}")

    def toggle_vibration(self, state):
        """Publishes a message to toggle the Vibration GPIO."""
        msg = Bool()
        msg.data = state
        self.vibration_publisher.publish(msg)
        self.get_logger().info(f"Vibration set to {'ON' if state else 'OFF'}")

    def move_dynamixel_motor(self, motor_id, position):
        """ Sends a command to move the Dynamixel motor to a specific position. """
        topic_name = None

        # Find the topic associated with the motor ID
        for key, settings in self.dynamixel_settings.items():
            if settings['motor_id'] == motor_id:
                topic_name = settings['topic']
                break

        if topic_name is None:
            self.get_logger().error(f"No topic found for Motor ID {motor_id}.")
            return

        # Publish the position command to the topic
        publisher = self.create_publisher(Float32, topic_name, 10)
        msg = Float32()
        msg.data = float(position)  # Ensure the position is a float
        publisher.publish(msg)
        self.get_logger().info(f"Published position {position} to Motor ID {motor_id} on topic '{topic_name}'.")

    def reboot_dynamixel_motor(self, motor_id):
        """Publishes a message to reboot the specified Dynamixel motor."""
        try:
            topic_name = None

            # Find the topic associated with the motor ID
            for key, settings in self.dynamixel_settings.items():
                if settings['motor_id'] == motor_id:
                    topic_name = settings['topic']
                    break

            if topic_name is None:
                self.get_logger().error(f"No topic found for Motor ID {motor_id}.")
                return

            # Publish a reboot command to the topic
            publisher = self.dynamixel_publishers.get(topic_name)
            if publisher:
                msg = Float32()
                msg.data = -1.0
                publisher.publish(msg)
                self.get_logger().info(f"Published reboot command to Motor ID {motor_id} on topic '{topic_name}'.")
        except Exception as e:
            self.get_logger().error(f"Exception during reboot of motor {motor_id}: {e}")

    def update_vesc_value(self, key, value):
        """ Updates the VESC motor value and publishes it """
        msg = Float32()
        msg.data = value
        try:
            self.vesc_settings[key]['publisher'].publish(msg)
            self.get_logger().info(f"Published to {key}: {value}")
        except Exception as e:
            self.get_logger().error(f"Error publishing to {key}: {e}")

    def set_vesc_to_zero(self, key, slider):
        """ Sets a specific VESC motor value to zero and updates the slider """
        self.update_vesc_value(key, 0.0)
        slider.set(0.0)  # Update the slider to reflect the zero value

    def set_all_vesc_to_zero(self):
        """Sets all VESC motor values to zero."""
        for key in self.vesc_settings.keys():
            self.update_vesc_value(key, 0.0)
        self.get_logger().info("All VESC motor values set to zero.")


def main():
    rclpy.init()
    try:
        node = ROS2GUINode()
    except Exception as e:
        print(f"Error initializing ROS2GUINode: {e}")
        rclpy.shutdown()


if __name__ == "__main__":
    main()