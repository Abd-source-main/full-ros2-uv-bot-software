"""Bring up the HC-SR04 ultrasonic driver (lib_hcsr04_ros).

REAL ROBOT ONLY. The driver links against wiringPi and calls wiringPiSetupGpio(),
so it only runs on a Raspberry Pi with the HC-SR04 physically wired to the GPIO
pins below. It will not run on a dev laptop or in Gazebo.

It publishes sensor_msgs/Range on:
    /HCSR04_ultrasonic/distance
    /HCSR04_ultrasonic/relative_velocity
in the `frame_id` frame (default base_link). For the Nav2 range costmap layer to
place obstacles correctly, that frame should sit where the sensor is mounted --
add an `ultrasonic_link` to the URDF (or a static_transform_publisher) and set
frame_id to it.

Standalone use:
    ros2 launch the_robot ultrasonic.launch.py trigger_pin:=14 echo_pin:=4
It is also included automatically by nav2.launch.py when use_ultrasonic:=true.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate the launch description."""
    # LaunchConfiguration substitutions resolve to strings, but the driver
    # declares these parameters as int / double / string. Wrap each with
    # ParameterValue(value_type=...) so launch coerces them to the right type;
    # otherwise the node rejects them with InvalidParameterType.
    trigger_pin = ParameterValue(LaunchConfiguration('trigger_pin'), value_type=int)
    echo_pin = ParameterValue(LaunchConfiguration('echo_pin'), value_type=int)
    frame_id = ParameterValue(LaunchConfiguration('frame_id'), value_type=str)
    minimum_range = ParameterValue(LaunchConfiguration('minimum_range'), value_type=float)
    maximum_range = ParameterValue(LaunchConfiguration('maximum_range'), value_type=float)
    field_of_view_deg = ParameterValue(
        LaunchConfiguration('field_of_view_deg'), value_type=float)

    return LaunchDescription([
        # BCM/GPIO pin numbers (wiringPiSetupGpio convention). Change to match
        # how you wired the sensor.
        DeclareLaunchArgument('trigger_pin', default_value='14'),
        DeclareLaunchArgument('echo_pin', default_value='4'),
        # TF frame the Range messages are stamped in (mount point of the sensor).
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        # Sensor limits, metres / degrees (HC-SR04: ~0.02-4.0 m, ~15 deg cone).
        DeclareLaunchArgument('minimum_range', default_value='0.03'),
        DeclareLaunchArgument('maximum_range', default_value='4.0'),
        DeclareLaunchArgument('field_of_view_deg', default_value='15.0'),

        Node(
            package='lib_hcsr04_ros',
            executable='RunUltrasonicHCSR04Wrapper',
            name='HCSR04_ultrasonic_driver',
            output='screen',
            parameters=[{
                'trigger_pin': trigger_pin,
                'echo_pin': echo_pin,
                'relative_to_frame_id': frame_id,
                'minimum_range': minimum_range,
                'maximum_range': maximum_range,
                'field_of_view_deg': field_of_view_deg,
            }],
        ),
    ])
