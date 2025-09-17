import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import tkinter as tk
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import csv
import os

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.create_subscription(Float32, 'temperature_data', self.temperature_callback, 10)
        self.create_subscription(Float32, 'humidity_data', self.humidity_callback, 10)
        self.sensor_window = SensorWindow()

    def temperature_callback(self, msg):
        temperature = round(msg.data, 2)
        self.sensor_window.update_temperature(temperature)

    def humidity_callback(self, msg):
        humidity = round(msg.data, 2)
        self.sensor_window.update_humidity(humidity)

    def run(self):
        """
        Start the Tkinter main loop to run the GUI.
        """
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(self)
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.start()
        self.sensor_window.run()
        ros_thread.join()


class SensorWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Sensor Data Display")

        self.temperature_label = tk.Label(self.root, text="Temperature: 0.00°C", font=("Arial", 16))
        self.temperature_label.pack(pady=10)

        self.humidity_label = tk.Label(self.root, text="Humidity: 0.00%", font=("Arial", 16))
        self.humidity_label.pack(pady=10)

        self.start_button = tk.Button(self.root, text="Start Recording", command=self.start_recording, font=("Arial", 16))
        self.start_button.pack(pady=10)

        self.stop_button = tk.Button(self.root, text="Stop Recording", command=self.stop_recording, font=("Arial", 16))
        self.stop_button.pack(pady=10)

        # Initialize matplotlib figure for temperature graph
        self.fig_temp, self.ax_temp = plt.subplots()
        self.ax_temp.set_title("Temperature")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")
        self.temp_data = []

        self.start_time = time.time()

        # Embed the matplotlib figure in Tkinter
        self.canvas_temp = FigureCanvasTkAgg(self.fig_temp, master=self.root)
        self.canvas_temp.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

        self.recording = False
        self.temp_csv_file = None
        self.temp_csv_writer = None

    def start_recording(self):
        self.recording = True
        trial_path = '~/science_ws/src/rover_science-master/rover_science/sensor-data/Trials/Trial1'
        os.makedirs(trial_path, exist_ok=True)
        self.temp_csv_file = open(os.path.join(trial_path, 'temperature_data.csv'), 'w', newline='')
        self.temp_csv_writer = csv.writer(self.temp_csv_file)
        self.temp_csv_writer.writerow(['Time (s)', 'Temperature (°C)'])

    def stop_recording(self):
        self.recording = False
        if self.temp_csv_file and not self.temp_csv_file.closed:
            self.temp_csv_file.close()
        trial_path = '~/science_ws/src/rover_science-master/rover_science/sensor-data/Trials/Trial1'
        self.fig_temp.savefig(os.path.join(trial_path, 'temperature_graph.png'))

    def update_temperature(self, temperature):
        self.temperature_label.config(text=f"Temperature: {temperature}°C")
        if self.recording:
            self.record_temperature(temperature)
            self.update_temperature_graph(temperature)

    def update_humidity(self, humidity):
        self.humidity_label.config(text=f"Humidity: {humidity}%")

    def record_temperature(self, temperature):
        current_time = time.time() - self.start_time
        self.temp_csv_writer.writerow([current_time, temperature])

    def update_temperature_graph(self, temperature):
        current_time = time.time() - self.start_time
        self.temp_data.append((current_time, temperature))
        times, temps = zip(*self.temp_data)
        self.ax_temp.clear()
        self.ax_temp.plot(times, temps, label="Temperature")
        self.ax_temp.set_title("Temperature")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")
        self.ax_temp.legend()
        self.canvas_temp.draw()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()