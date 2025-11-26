from setuptools import find_packages, setup

package_name = 'img_calc_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # instalar launch files para que ros2 los encuentre
        ('share/' + package_name + '/launch', ['launch/launch_papa_orientation.py']),
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
            'papa_orientation = img_calc_pkg.papa_orientation:main',
        ],
    },
)
