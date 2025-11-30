from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinematics_pkg',
            executable='inv_kinematics',
            name='inv_kinematics_node',
            output='screen',
            emulate_tty=True
        ),

    ])
