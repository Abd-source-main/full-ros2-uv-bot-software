"""Real-robot base bringup: TF tree + wheel odometry (NO Gazebo).

This is the real-hardware counterpart to sim.launch.py. It brings up the two
things the hardware needs for Nav2 to have a complete TF chain when there is no
LiDAR/AMCL:

  * robot_state_publisher -- publishes base_footprint -> base_link -> base_scan ->
    wheels from the URDF (static part of the TF tree), plus /robot_description.
  * wheel_odometry        -- publishes /odom and the odom -> base_footprint TF
                             (dynamic part). See wheel_odometry.py for the
                             `source` param (ticks vs cmd_vel).

Combined with the map -> odom static transform from nav2.launch.py
(localization:=none), this yields the full map -> odom -> base_footprint -> ...
chain without a laser scanner.

It does NOT start motor/sensor drivers -- add your base driver (and the
ultrasonic via ultrasonic.launch.py) alongside it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    """Resolve the URDF choice at launch time, then build the nodes."""
    odom_source = LaunchConfiguration('odom_source')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev')
    urdf = LaunchConfiguration('urdf').perform(context)

    # Which robot description to publish. 'my_robot' = this package's custom
    # differential-drive URDF (edit urdf/my_robot.urdf.xacro with your real
    # measurements). 'waffle' = the TurtleBot3 Waffle (only correct if your
    # robot actually is a Waffle). Both use the same frame names, so nothing
    # downstream changes.
    if urdf == 'waffle':
        urdf_xacro = os.path.join(
            get_package_share_directory('turtlebot3_description'),
            'urdf', 'turtlebot3_waffle.urdf',
        )
    else:
        urdf_xacro = os.path.join(
            get_package_share_directory('the_robot'),
            'urdf', 'my_robot.urdf.xacro',
        )
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro]), value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description,
        }],
    )

    wheel_odometry = Node(
        package='the_robot',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'source': odom_source,
            'wheel_radius': ParameterValue(wheel_radius, value_type=float),
            'wheel_separation': ParameterValue(wheel_separation, value_type=float),
            'ticks_per_rev': ParameterValue(ticks_per_rev, value_type=int),
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
    )

    return [robot_state_publisher, wheel_odometry]


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf', default_value='my_robot', choices=['my_robot', 'waffle'],
            description="Robot description: 'my_robot' (this package's custom "
                        "URDF -- edit urdf/my_robot.urdf.xacro) or 'waffle' "
                        "(TurtleBot3 Waffle).",
        ),
        DeclareLaunchArgument(
            'odom_source', default_value='cmd_vel',
            description="Odometry input: 'ticks' (real encoders on wheel_ticks) "
                        "or 'cmd_vel' (open-loop, testing only).",
        ),
        DeclareLaunchArgument('wheel_radius', default_value='0.033'),
        DeclareLaunchArgument('wheel_separation', default_value='0.16'),
        DeclareLaunchArgument('ticks_per_rev', default_value='4096'),
        OpaqueFunction(function=launch_setup),
    ])
