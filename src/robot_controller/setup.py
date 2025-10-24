from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giovanni',
    maintainer_email='giovanni@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtlebot_controller_node = robot_controller.isc_control_law_decoupling:main",
            "create_trajectory_node = robot_controller.create_trajectory:main",
            "turtlesim_controller_node = robot_controller.isc_control_law_decoupling_turtlesim:main",
            "waypoint_server = robot_controller.waypoint_server:main"
        ],
    },
)
