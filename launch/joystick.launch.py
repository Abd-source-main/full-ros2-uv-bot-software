"""Joystick web driving -- real robot, NO SLAM.

Drive the robot from a browser joystick (or the W/A/S/D keys). There is no
mapping here, so this runs on a bare Raspberry Pi with no Gazebo and no
slam_toolbox -- only the driving chain comes up:

    web_teleop    -- http://<pi-ip>:8081 : joystick / WASD -> /cmd_vel
    guarded_base  -- HC-SR04 -> collision_guard -> /cmd_vel_safe -> motor_driver

The map panel in the web page just stays blank (no /map is published); the
joystick still drives the robot.

Usage
-----
    ros2 launch the_robot joystick.launch.py                 # with crash guard
    ros2 launch the_robot joystick.launch.py guard:=false    # motors only, no HC-SR04

When guard:=false the ultrasonic + collision_guard are skipped and the motor
driver listens directly on /cmd_vel (use only if the sensor isn't wired).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('the_robot')
    guard = LaunchConfiguration('guard').perform(context).lower() in ('true', '1')

    actions = []

    if guard:
        # Full safe base: HC-SR04 -> collision_guard -> motor_driver(cmd_vel_safe).
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'guarded_base.launch.py')
            ),
        ))
    else:
        # No sensor: drive the motors straight from /cmd_vel.
        actions.append(Node(
            package='the_robot',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
        ))

    # Web joystick app -> /cmd_vel. use_sim_time is false: this is the real robot.
    actions.append(Node(
        package='the_robot',
        executable='web_teleop',
        name='web_teleop',
        output='screen',
        parameters=[{'use_sim_time': False}],
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'guard', default_value='true',
            description='Bring up the HC-SR04 + collision_guard crash-protection '
                        'chain. Set false to drive the motors directly from '
                        '/cmd_vel (no ultrasonic).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
