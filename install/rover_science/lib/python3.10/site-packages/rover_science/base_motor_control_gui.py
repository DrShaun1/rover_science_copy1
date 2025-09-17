import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import tkinter as tk


class ROS2GUINode(Node):
    def __init__(self):
        super().__init__('ros2_gui')

        # VESC motor settings
        self.vesc_settings = {
            'pump': {'current': 0.8, 'publisher': self.create_publisher(Float32, 'pump', 10)},
            'dmotor1': {'duty_cycle': 0.7, 'publisher': self.create_publisher(Float32, 'dmotor1', 10)},
            'dmotor2': {'duty_cycle': 0.7, 'publisher': self.create_publisher(Float32, 'dmotor2', 10)},
            'inductor': {'duty_cycle': 1.0, 'publisher': self.create_publisher(Float32, 'inductor', 10)},
            'linear1': {'duty_cycle': 0.1, 'publisher': self.create_publisher(Float32, 'linear1', 10)},
            'linear2': {'duty_cycle': 0.1, 'publisher': self.create_publisher(Float32, 'linear2', 10)},
            'mixer': {'duty_cycle': 1.0, 'publisher': self.create_publisher(Float32, 'mixer', 10)},
        }

        # Dynamixel motor settings
        self.dynamixel_settings = {
            'Purge2': {'min_position': 1536, 'max_position': 2002, 'publisher': self.create_publisher(Float32, 'Purge2', 10)},
            'Splitter': {'min_position': 0, 'max_position': 320, 'publisher': self.create_publisher(Float32, 'Splitter', 10)},
            'Purge1': {'min_position': 3200, 'max_position': 4100, 'publisher': self.create_publisher(Float32, 'Purge1', 10)},
            'SampleCache': {'min_position': 495, 'max_position': 2582, 'publisher': self.create_publisher(Float32, 'SampleCache', 10)},
            'Sample1': {'min_position': 100, 'max_position': 3000, 'publisher': self.create_publisher(Float32, 'Sample1', 10)},
        }

        # Initialize the GUI
        self.root = tk.Tk()
        self.root.title("ROS2 Motor Control GUI")

        # Create frames for VESC and Dynamixel motors
        vesc_frame = tk.LabelFrame(self.root, text="VESC Motors", padx=10, pady=10)
        vesc_frame.grid(row=0, column=0, padx=10, pady=10, sticky="n")

        dynamixel_frame = tk.LabelFrame(self.root, text="Dynamixel Motors", padx=10, pady=10)
        dynamixel_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")

        # Add VESC motor controls
        for key, settings in self.vesc_settings.items():
            frame = tk.Frame(vesc_frame)
            frame.pack(pady=5)

            tk.Label(frame, text=key).pack()

            if key == 'pump':  # Current motor
                slider = tk.Scale(frame, from_=0, to=settings['current'], resolution=0.01, orient=tk.HORIZONTAL,
                                  command=lambda val, k=key: self.update_vesc_value(k, float(val)))
                label = tk.Label(frame, text="Current: 0.00A")
            else:  # Duty cycle motors
                max_duty_cycle = settings['duty_cycle']
                slider = tk.Scale(frame, from_=-max_duty_cycle, to=max_duty_cycle, resolution=0.001, orient=tk.HORIZONTAL,
                                  command=lambda val, k=key: self.update_vesc_value(k, float(val)))
                label = tk.Label(frame, text=f"Duty Cycle: 0.00 (range -{max_duty_cycle} to {max_duty_cycle})")

            slider.pack()
            label.pack()

            # Add Zero button for each motor
            zero_button = tk.Button(frame, text="Zero", command=lambda k=key: self.update_vesc_value(k, 0.0))
            zero_button.pack(pady=5)

        # Add Zero All button for all VESC motors
        zero_all_button = tk.Button(vesc_frame, text="Zero All", command=self.zero_all_vesc_motors)
        zero_all_button.pack(pady=10)

        # Add Dynamixel motor controls
        for key, settings in self.dynamixel_settings.items():
            frame = tk.Frame(dynamixel_frame)
            frame.pack(pady=5)

            tk.Label(frame, text=key).pack()

            min_position = settings['min_position']
            max_position = settings['max_position']

            if key == 'Splitter':  # Special case for Splitter
                min_button_label = "Inductor"
                max_button_label = "Storage"
            elif key == 'Purge1':  # Special case for Purge1
                min_button_label = f"Close ({min_position})"
                max_button_label = f"Open ({max_position})"
            else:  # Default case
                min_button_label = f"Open ({min_position})"
                max_button_label = f"Close ({max_position})"

            # Button for minimum position
            min_button = tk.Button(frame, text=min_button_label,
                                    command=lambda k=key, pos=min_position: self.update_dynamixel_value(k, pos))
            min_button.pack(side=tk.LEFT, padx=5)

            # Button for maximum position
            max_button = tk.Button(frame, text=max_button_label,
                                    command=lambda k=key, pos=max_position: self.update_dynamixel_value(k, pos))
            max_button.pack(side=tk.LEFT, padx=5)

        # Start the Tkinter loop
        self.root.mainloop()

    def update_vesc_value(self, key, value):
        """ Updates the VESC motor value and publishes it """
        msg = Float32()
        msg.data = value
        try:
            self.vesc_settings[key]['publisher'].publish(msg)
            print(f"Published to {key}: {value}")
        except Exception as e:
            print(f"Error publishing to {key}: {e}")

    def zero_all_vesc_motors(self):
        """ Sets all VESC motor values to zero """
        for key in self.vesc_settings.keys():
            self.update_vesc_value(key, 0.0)

    def update_dynamixel_value(self, key, value):
        """ Updates the Dynamixel motor value and publishes it """
        try:
            value = float(value)  # Ensure value is a float
        except (ValueError, TypeError):
            print(f"Invalid value for {key}: {value}")
            return

        msg = Float32()
        msg.data = value  # Assign the float value to msg.data
        try:
            self.dynamixel_settings[key]['publisher'].publish(msg)
            print(f"Published to {key}: {value}")
        except Exception as e:
            print(f"Error publishing to {key}: {e}")


def main():
    rclpy.init()
    try:
        node = ROS2GUINode()
    except Exception as e:
        print(f"Error initializing ROS2GUINode: {e}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()