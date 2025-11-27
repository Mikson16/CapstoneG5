from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo que levanta la comunicacion con el arduino
        Node(
            package='comunication_pkg',
            executable='arduino_com',
            name='arduino_com_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='comunication_pkg',
            executable='arduino_coord_pub',
            name='arduino_coord_pub_node',
            output='screen',
            emulate_tty=True,
        )
    ])
