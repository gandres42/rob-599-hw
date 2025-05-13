#!/python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    pkg_name = 'hw5'
    config_path = os.path.join(
        get_package_share_path(pkg_name),
        'rviz',
        'config.rviz'
    )
    print(config_path)
    return LaunchDescription([        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_path]
        ),
        Node(
            package='hw5',
            executable='filter',
        ),
        Node(
            package='hw5',
            executable='detect',
        ),
    ])
