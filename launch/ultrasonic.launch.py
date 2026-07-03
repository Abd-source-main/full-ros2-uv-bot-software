"""Bring up the HC-SR04 ultrasonic driver (gpiozero-based, in this package).

REAL ROBOT ONLY. Needs the Raspberry Pi GPIO + gpiozero (same as motor_driver);
it will not run on a dev laptop or in Gazebo.

It publishes sensor_msgs/Range on:
    /HCSR04_ultrasonic/distance
in the `frame_id` frame (default base_link). For a Nav2 range costmap layer to
place obstacles correctly, that frame should sit where the sensor is mounted --
add an `ultrasonic_link` to the URDF (or a static_transform_publisher) and set
frame_id to it.

Pins are BCM/GPIO numbers. This robot: TRIG=GPIO16 (physical 36),
ECHO=GPIO13 (physical 33, through a 5V->3.3V voltage divider).

Standalone use:
    ros2 launch the_robot ultrasonic.launch.py trigger_pin:=16 echo_pin:=13
It is also included by guarded_base.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate the launch description."""
    # LaunchConfiguration substitutions resolve to strings; the node declares
    # these as int / double / string, so wrap each with ParameterValue to coerce
    # to the right type (otherwise the node rejects them).
    trigger_pin = ParameterValue(LaunchConfiguration('trigger_pin'), value_type=int)
    echo_pin = ParameterValue(LaunchConfiguration('echo_pin'), value_type=int)
    frame_id = ParameterValue(LaunchConfiguration('frame_id'), value_type=str)
    minimum_range = ParameterValue(LaunchConfiguration('minimum_range'), value_type=float)
    maximum_range = ParameterValue(LaunchConfiguration('maximum_range'), value_type=float)
    field_of_view_deg = ParameterValue(
        LaunchConfiguration('field_of_view_deg'), value_type=float)

    return LaunchDescription([
        # BCM/GPIO pin numbers. TRIG=GPIO16 (phys 36), ECHO=GPIO13 (phys 33).
        DeclareLaunchArgument('trigger_pin', default_value='16'),
        DeclareLaunchArgument('echo_pin', default_value='13'),
        # TF frame the Range messages are stamped in (mount point of the sensor).
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        # Sensor limits, metres / degrees (HC-SR04: ~0.02-4.0 m, ~15 deg cone).
        DeclareLaunchArgument('minimum_range', default_value='0.03'),
        DeclareLaunchArgument('maximum_range', default_value='4.0'),
        DeclareLaunchArgument('field_of_view_deg', default_value='15.0'),

        Node(
            package='the_robot',
            executable='ultrasonic_hcsr04',
            name='ultrasonic_hcsr04',
            output='screen',
            parameters=[{
                'trigger_pin': trigger_pin,
                'echo_pin': echo_pin,
                'frame_id': frame_id,
                'minimum_range': minimum_range,
                'maximum_range': maximum_range,
                'field_of_view_deg': field_of_view_deg,
            }],
        ),
    ])
