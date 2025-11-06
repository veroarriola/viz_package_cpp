from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, FileContent, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
            executable='move_node',
            name='move_node',
            output='screen',
        ),
        # Publica las transformaciones estáticas del modelo
        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'publish_frequency': 30.0,
            }],
            arguments=[FileContent(
                PathJoinSubstitution([FindPackageShare('viz_package_cpp'), 'urdf', 'paquito.urdf']))]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_dir, 'rviz', 'panel.rviz')],
            output='screen'
        ),
        Node(
            package='viz_package_cpp',
            executable='circle_control_node',
            name='circle_control_node',
            output='screen',
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge',
            output='screen'
        ),
    ])

