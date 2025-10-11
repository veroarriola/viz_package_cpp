from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    package_dir = get_package_share_directory('viz_package_cpp')
    path_to_urdf = os.path.join(package_dir, 'urdf', 'paquito.urdf')
    #with open(path_to_urdf, 'r') as f:
    #    robot_desc = f.read()
    robot_desc = ParameterValue(
                    Command(['xacro ', str(path_to_urdf)]), value_type=str
                )
    return LaunchDescription([
        Node(
            package='viz_package_cpp',
            #namespace='paquito1',
            executable='ps2_joy_node',
            name='ps2_hoy',
            output='screen',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        )
    ])

