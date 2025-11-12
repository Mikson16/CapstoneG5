from setuptools import find_packages, setup

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #instalar launch files para que ros2 los encuentre
        ('share/' + package_name + '/launch', ['launch/launch_static_camera.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joseq',
    maintainer_email='joseq@todo.todo',
    description='Camera package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'static_camera = camera_pkg.static_camera:main',
            'static_camera_papa = camera_pkg.static_camera_papa:main',
            'static_camera_robot = camera_pkg.static_camera_robot:main',
        ],
    },
)
