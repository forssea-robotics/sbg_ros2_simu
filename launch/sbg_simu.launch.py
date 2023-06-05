from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sbg_simu_dir = get_package_share_directory('sbg_simu')

    return LaunchDescription([
        Node(
            package='sbg_simu',
            executable='sbg_simu',
            output='screen',
            name='sbg_simu'
        ),
        Node(
            package='sbg_driver',
            executable='sbg_device',
            output='screen',
            name='sbg_device',
            parameters=[sbg_simu_dir + '/config/sbg_simu.yaml']
        )
    ])
