from setuptools import find_packages, setup

package_name = 'thor_status_publisher'

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
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'robot_status_publisher = thor_status_publisher.robot_status_publisher:main',
        'joint_goal_listener = thor_status_publisher.joint_goal_listener:main',
        'random_position_commander = thor_status_publisher.random_position_commander:main',
        'ik_goal_listener = thor_status_publisher.ik_goal_listener:main',
        'cartesian_goal_listener = thor_status_publisher.cartesian_goal_listener:main',
        ],
    },
)
