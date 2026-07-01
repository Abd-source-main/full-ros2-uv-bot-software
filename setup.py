import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'the_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps', 'worlds'),
            glob('maps/worlds/*.world')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eng-abd',
    maintainer_email='eng-abd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_gazebo = the_robot.run_gazebo:main',
            'spawn_robot = the_robot.spawn_robot:main',
            'teleop_wasd = the_robot.teleop_wasd:main',
            'nav2_goal = the_robot.nav2_goal:main',
            'web_goal = the_robot.web_goal:main',
            'slam = the_robot.slam:main',
        ],
    },
)
