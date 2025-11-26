from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='img_calc_pkg',
            executable='papa_orientation',
            name='papa_orientation_node',
            output='screen',
            emulate_tty=True
        )

    ])
