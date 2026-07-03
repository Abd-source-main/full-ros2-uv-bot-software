"""Real-robot safe base: HC-SR04 -> collision_guard -> motor_driver.

REAL ROBOT ONLY (needs the Raspberry Pi GPIO + the lib_hcsr04_ros driver).

Brings up the crash-protection chain:

    HC-SR04 driver   -> /HCSR04_ultrasonic/distance (sensor_msgs/Range)
    collision_guard  -> reads /cmd_vel + the range, publishes /cmd_vel_safe
    motor_driver     -> subscribes /cmd_vel_safe (remapped) and drives the L298N

Your command source (teleop_wasd, Nav2, web_teleop, ...) keeps publishing to the
normal /cmd_vel. The guard blocks forward motion whenever the ultrasonic sees an
obstacle closer than `stop_distance`, so the robot stops before it crashes and
stays stopped until the way is clear -- reverse and turning still pass through so
you can back out.

The HC-SR04 driver itself is not duplicated here: this reuses ultrasonic.launch.py
(single source of truth for the sensor pins/range), so don't run that file too --
it would start the driver twice on the same GPIO pins.

    ros2 launch the_robot guarded_base.launch.py
    ros2 launch the_robot guarded_base.launch.py stop_distance:=0.3 trigger_pin:=14 echo_pin:=4
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate the launch description."""
    pkg_share = get_package_share_directory('the_robot')

    stop_distance = ParameterValue(LaunchConfiguration('stop_distance'), value_type=float)
    clear_distance = ParameterValue(LaunchConfiguration('clear_distance'), value_type=float)

    return LaunchDescription([
        # HC-SR04 wiring (BCM/GPIO numbering) -- forwarded to ultrasonic.launch.py,
        # which owns the sensor's default pins/range.
        # TRIG=GPIO16 (physical pin 36), ECHO=GPIO13 (physical pin 33).
        DeclareLaunchArgument('trigger_pin', default_value='16'),
        DeclareLaunchArgument('echo_pin', default_value='13'),
        # Guard thresholds (metres). clear_distance must be > stop_distance.
        DeclareLaunchArgument('stop_distance', default_value='0.25'),
        DeclareLaunchArgument('clear_distance', default_value='0.35'),

        # 1) HC-SR04 ultrasonic driver -> /HCSR04_ultrasonic/distance.
        #    Reuse ultrasonic.launch.py rather than re-declaring the node, so the
        #    sensor config lives in exactly one place.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'ultrasonic.launch.py')),
            launch_arguments={
                'trigger_pin': LaunchConfiguration('trigger_pin'),
                'echo_pin': LaunchConfiguration('echo_pin'),
            }.items(),
        ),

        # 2) Collision guard: /cmd_vel + range -> /cmd_vel_safe
        Node(
            package='the_robot',
            executable='collision_guard',
            name='collision_guard',
            output='screen',
            parameters=[{
                'stop_distance': stop_distance,
                'clear_distance': clear_distance,
                'range_topic': '/HCSR04_ultrasonic/distance',
            }],
        ),

        # 3) Motor driver, listening to the *guarded* command topic.
        Node(
            package='the_robot',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
            remappings=[('cmd_vel', 'cmd_vel_safe')],
        ),
    ])
