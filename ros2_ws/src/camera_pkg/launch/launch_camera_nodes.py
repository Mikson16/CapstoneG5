from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo que levanta la camara estatica
        Node(
            package='camera_pkg',
            executable='static_camera', 
            name='static_camera_node',
            output='screen',
            emulate_tty=True, # si necesito una salida interacitva o interaccion con terminal usar true, sino puedo colocarlo en false
            parameters=[{'headless': False}] # Cambiar a True para no desplegar las ventanas
        ),

        # Nodo de procesamiento de la imagen
        Node(
            package='camera_pkg',
            executable='static_camera_papa',
            name='static_camera_papa_node',
            output='screen',
            emulate_tty=True,
        ),
        
        Node(
            package='camera_pkg',
            executable='static_camera_robot',
            name='static_camera_robot_node',
            output='screen',
            emulate_tty= True,
        ),
        Node(
            package='camera_pkg',
            executable='static_camera_emergency',
            name='static_camera_emergency_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
