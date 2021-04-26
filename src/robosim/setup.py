from setuptools import setup

package_name = 'robosim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/robosim_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohak Patel',
    maintainer_email='mohak.patel2312@gmail.com',
    description='Controlling jetson car using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'talker = robosim.robot_controller:main',
                'listener = robosim.robot_simulator:main',
        ],
    },
)
