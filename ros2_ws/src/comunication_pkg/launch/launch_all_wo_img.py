from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # (pkg, filename, delay_seconds)
    launches = [
        # ('camera_pkg', 'launch_camera_nodes.py', 0.0),
        ('comunication_pkg', 'launch_arduino_com.py', 4.0),   # arrancar 5s despues
        ('img_calc_pkg', 'launch_calc_nodes.py', 2.0),        # arrancar 2s despues
        ('kinematics_pkg', 'launch_inv_kinematics.py', 3.0), # arrancar 6s despues
    ]

    for pkg, fname, delay in launches:
        path = os.path.join(get_package_share_directory(pkg), 'launch', fname)
        if os.path.exists(path):
            include = IncludeLaunchDescription(PythonLaunchDescriptionSource(path))
            if delay and delay > 0.0:
                ld.add_action(TimerAction(period=delay, actions=[include]))
            else:
                ld.add_action(include)
        else:
            print(f'[launch_all] No encontrado: {path}')

    return ld
