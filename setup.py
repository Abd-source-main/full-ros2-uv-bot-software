from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'the_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eng-abd',
    maintainer_email='eng-abd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = the_robot.motor_driver:main',
            'nav2_goal = the_robot.nav2_goal:main',
            'run_gazebo = the_robot.run_gazebo:main',
            'saif = the_robot.saif:main',
            'slam = the_robot.slam:main',
            'spawn_robot = the_robot.spawn_robot:main',
            'teleop_wasd = the_robot.teleop_wasd:main',
            'web_goal = the_robot.web_goal:main',
            'wheel_odometry = the_robot.wheel_odometry:main',
        ],
    },
)
