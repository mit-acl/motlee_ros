import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motlee_ros2',
            executable='mot_node',
            name='mot',
            output='screen',
            emulate_tty=True,
            parameters=[os.path.join(get_package_share_directory('motlee_ros2'), 'cfg', 'mot.yaml')]
        )
    ])