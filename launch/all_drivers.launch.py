from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_science',
            executable='rover_vesc_driver',
            name='vesc_driver_node',
            output='screen'
        ),
        Node(
            package='rover_science',
            executable='rover_dynamixel_driver',
            name='dynamixel_driver_node',
            output='screen'
        ),
        Node(
            package='rover_science',
            executable='rover_temp_and_humidity_drivers',
            name='temp_and_humidity_driver_node',
            output='screen'
        ),
        Node(
            package='rover_science',
            executable='rover_carbon_sensor_driver',
            name='carbon_sensor_driver_node',
            output='screen'
        ),
        Node(
            package='rover_science',
            executable='gpio_switch_driver',
            name='gpio_switch_driver_node',
            output='screen'
        ),
    ])
