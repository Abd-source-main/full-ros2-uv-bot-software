"""Launch Gazebo with the hospital world, spawn the robot, and publish its TF tree."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate the launch description."""
    # Waffle URDF (xacro) shipped with turtlebot3_description. Processing it with
    # namespace="" yields the frames base_footprint / base_link / base_scan, which
    # match the spawned SDF and the AMCL base_frame_id.
    urdf_xacro = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_waffle.urdf',
    )
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro]), value_type=str
    )

    # 0) Publish the rrobot_descriptionobot's static TF tree (base_footprint -> base_link ->
    #    base_scan, wheels, imu_link, ...) from the URDF, plus /robot_description.
    #    Gazebo's plugins only publish odom -> base_footprint and the sensor
    #    topics; without robot_state_publisher the laser frame (base_scan) is not
    #    connected to the TF tree, so AMCL cannot transform the scan and never
    #    localizes, and RViz has no robot model to display.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )

    # 1) Start Gazebo with the hospital world.
    run_gazebo_node = Node(
        package='the_robot',
        executable='run_gazebo',
        name='run_gazebo',
        output='screen',
    )

    # 2) Spawn the robot. Delay it so gzserver is up and ready to accept
    #    the spawn_entity request; otherwise the spawn silently fails.
    spawn_robot_node = Node(
        package='the_robot',
        executable='spawn_robot',
        name='spawn_robot',
        output='screen',
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot_node],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        run_gazebo_node,
        delayed_spawn,
    ])


def main(args=None):
    ld = generate_launch_description()
    return ld


if __name__ == "__main__":
    main()
