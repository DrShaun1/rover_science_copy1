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
        self.create_subscription(Float32, 'thermocouple1_data', self.thermocouple1_callback, 10)
        self.create_subscription(Float32, 'thermocouple2_data', self.thermocouple2_callback, 10)
        self.create_subscription(Float32, 'soil_moisture_data', self.soil_moisture_callback, 10)
        self.create_subscription(Float32, 'co2_data', self.co2_callback, 10)
        self.create_subscription(Float32, 'scd41_temperature_data', self.scd41_temperature_callback, 10)
        self.create_subscription(Float32, 'humidity_data', self.humidity_callback, 10)
        self.sensor_window = SensorWindow()

    def thermocouple1_callback(self, msg):
        temperature = round(msg.data, 2)
        self.sensor_window.update_thermocouple1(temperature)

    def thermocouple2_callback(self, msg):
        temperature = round(msg.data, 2)
        self.sensor_window.update_thermocouple2(temperature)

    def soil_moisture_callback(self, msg):
        moisture = round(msg.data, 2)
        self.sensor_window.update_soil_moisture(moisture)

    def co2_callback(self, msg):
        co2 = round(msg.data, 2)
        self.sensor_window.update_co2(co2)
        if self.sensor_window.recording:
            self.sensor_window.record_co2(co2)
        self.sensor_window.update_co2_graph(co2)

    def scd41_temperature_callback(self, msg):
        temperature = round(msg.data, 2)
        self.sensor_window.update_scd41_temperature(temperature)
        if self.sensor_window.recording:
            self.sensor_window.record_temperature(temperature)
        self.sensor_window.update_temperature_graph(temperature)

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

        self.thermocouple1_label = tk.Label(self.root, text="Thermocouple 1: 10.00°C", font=("Arial", 16))
        self.thermocouple1_label.pack(pady=10)

        self.thermocouple2_label = tk.Label(self.root, text="Thermocouple 2: 4.00°C", font=("Arial", 16))
        self.thermocouple2_label.pack(pady=10)

        self.soil_moisture_label = tk.Label(self.root, text="Soil Moisture: 442.00", font=("Arial", 16))
        self.soil_moisture_label.pack(pady=10)

        self.co2_label = tk.Label(self.root, text="CO2: 0.00 ppm", font=("Arial", 16))
        self.co2_label.pack(pady=10)

        self.scd41_temperature_label = tk.Label(self.root, text="SCD41 Temperature: 0.00°C", font=("Arial", 16))
        self.scd41_temperature_label.pack(pady=10)

        self.humidity_label = tk.Label(self.root, text="Humidity: 21.00%", font=("Arial", 16))
        self.humidity_label.pack(pady=10)

        self.start_button = tk.Button(self.root, text="Pyrolysis Test Start", command=self.start_recording, font=("Arial", 16))
        self.start_button.pack(pady=10)

        self.stop_button = tk.Button(self.root, text="Pyrolysis Test Stop", command=self.stop_recording, font=("Arial", 16))
        self.stop_button.pack(pady=10)

        # Initialize matplotlib figures for temperature and CO2 graphs
        self.fig_temp, self.ax_temp = plt.subplots()
        self.ax_temp.set_title("SCD41 Temperature")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")
        self.temp_data = []

        self.fig_co2, self.ax_co2 = plt.subplots()
        self.ax_co2.set_title("CO2 Levels")
        self.ax_co2.set_xlabel("Time (s)")
        self.ax_co2.set_ylabel("CO2 (ppm)")
        self.co2_data = []

        self.start_time = time.time()

        # Embed the matplotlib figures in Tkinter
        self.canvas_temp = FigureCanvasTkAgg(self.fig_temp, master=self.root)
        self.canvas_temp.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

        self.canvas_co2 = FigureCanvasTkAgg(self.fig_co2, master=self.root)
        self.canvas_co2.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

        self.recording = False
        self.temp_csv_file = None
        self.co2_csv_file = None
        self.temp_csv_writer = None
        self.co2_csv_writer = None

    def start_recording(self):
        self.recording = True
        trial_path = '/~/science_ws/src/rover_science-master/rover_science/sensor-data/Trials/Trial1'
        os.makedirs(trial_path, exist_ok=True)
        self.temp_csv_file = open(os.path.join(trial_path, 'scd41_temperature_data.csv'), 'w', newline='')
        self.co2_csv_file = open(os.path.join(trial_path, 'co2_data.csv'), 'w', newline='')
        self.temp_csv_writer = csv.writer(self.temp_csv_file)
        self.co2_csv_writer = csv.writer(self.co2_csv_file)
        self.temp_csv_writer.writerow(['Time (s)', 'Temperature (°C)'])
        self.co2_csv_writer.writerow(['Time (s)', 'CO2 (ppm)'])

    def stop_recording(self):
        self.recording = False
        if self.temp_csv_file and not self.temp_csv_file.closed:
            self.temp_csv_file.close()
        if self.co2_csv_file and not self.co2_csv_file.closed:
            self.co2_csv_file.close()
        trial_path = '/~/science_ws/src/rover_science-master/rover_science/sensor-data/Trials/Trial1'
        self.fig_temp.savefig(os.path.join(trial_path, 'scd41_temperature_graph.png'))
        self.fig_co2.savefig(os.path.join(trial_path, 'co2_graph.png'))

    def record_temperature(self, temperature):
        current_time = time.time() - self.start_time
        self.temp_csv_writer.writerow([current_time, temperature])

    def record_co2(self, co2):
        current_time = time.time() - self.start_time
        self.co2_csv_writer.writerow([current_time, co2])

    def update_thermocouple1(self, temperature):
        self.thermocouple1_label.config(text=f"Thermocouple 1: {temperature}°C")

    def update_thermocouple2(self, temperature):
        self.thermocouple2_label.config(text=f"Thermocouple 2: {temperature}°C")

    def update_soil_moisture(self, moisture):
        self.soil_moisture_label.config(text=f"Soil Moisture: {moisture}")

    def update_co2(self, co2):
        self.co2_label.config(text=f"CO2: {co2} ppm")

    def update_scd41_temperature(self, temperature):
        self.scd41_temperature_label.config(text=f"SCD41 Temperature: {temperature}°C")

    def update_humidity(self, humidity):
        self.humidity_label.config(text=f"Humidity: {humidity}%")

    def update_temperature_graph(self, temperature):
        current_time = time.time() - self.start_time
        self.temp_data.append((current_time, temperature))
        times, temps = zip(*self.temp_data)
        self.ax_temp.clear()
        self.ax_temp.plot(times, temps, label="SCD41 Temperature")
        self.ax_temp.set_title("SCD41 Temperature")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")
        self.ax_temp.legend()
        self.canvas_temp.draw()

    def update_co2_graph(self, co2):
        current_time = time.time() - self.start_time
        self.co2_data.append((current_time, co2))
        times, co2_levels = zip(*self.co2_data)
        self.ax_co2.clear()
        self.ax_co2.plot(times, co2_levels, label="CO2")
        self.ax_co2.set_title("CO2 Levels")
        self.ax_co2.set_xlabel("Time (s)")
        self.ax_co2.set_ylabel("CO2 (ppm)")
        self.ax_co2.legend()
        self.canvas_co2.draw()

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