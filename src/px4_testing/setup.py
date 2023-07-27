from setuptools import setup

package_name = 'px4_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "keyboard_control = px4_testing.keyboard_control:main",
            "takeoff = px4_testing.takeoff:main",
            "square = px4_testing.square:main",
            "circle = px4_testing.circle:main",
            "waypoint = px4_testing.trajectory_waypoint:main",
            "keyboard_listener = px4_testing.keyboard_listener:main",
            "example_publisher = px4_testing.examplePublisher:main",
            "example_subscriber = px4_testing.exampleSubscriber:main"
        ],
    },
)