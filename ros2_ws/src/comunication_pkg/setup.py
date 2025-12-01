from setuptools import find_packages, setup

package_name = 'comunication_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #Instalar launch files para que ros2 los encuentre
        ('share/' + package_name + '/launch', ['launch/launch_arduino_com.py']),
        ('share/' + package_name + '/launch', ['launch/launch_all_wo_img.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joseq',
    maintainer_email='jquimi@uc.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arduino_com = comunication_pkg.arduino_com:main',
            'arduino_coord_pub = comunication_pkg.arduino_coord_pub:main',
        ],
    },
)
