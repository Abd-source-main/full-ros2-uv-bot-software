"""Drive-and-map: web driving app + slam_toolbox, mapping as you go.

Brings up everything needed to drive the robot from a browser while SLAM builds
the map, and continues a previously-saved map on relaunch:

  * base            -- Gazebo + robot + TF (sim) or the real base bringup.
  * slam            -- the_robot's slam node: runs slam_toolbox, and on exit
                       serializes the pose graph (so the NEXT launch continues
                       this map) and saves pgm/yaml (for web_goal / nav2).
  * web_teleop      -- http://localhost:8081 : hold buttons / WASD to drive,
                       watch the map build live.
  * rviz (optional) -- slam_toolbox's default view.

Usage
-----
  ros2 launch the_robot slam_web.launch.py                 # sim, fresh or continued map
  ros2 launch the_robot slam_web.launch.py rviz:=true      # also open RViz
  ros2 launch the_robot slam_web.launch.py sim:=false use_sim_time:=false   # real robot

Exit with Ctrl+C: the map is serialized + saved automatically. Launch again and
mapping resumes in the saved map (the robot must start at its original origin).
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
    the_robot_dir = get_package_share_directory('the_robot')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context)
    sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1')
    rviz = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1')
    sim_time = use_sim_time.lower() in ('true', '1')

    actions = []

    # 1) Base + TF. In sim: Gazebo + robot spawn + robot_state_publisher.
    #    On a real robot: the base bringup (robot_state_publisher + wheel odom).
    base_launch = 'sim.launch.py' if sim else 'robot_bringup.launch.py'
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(the_robot_dir, 'launch', base_launch)
        ),
    ))

    # 2) SLAM manager (runs slam_toolbox; saves + continues the map).
    actions.append(Node(
        package='the_robot',
        executable='slam',
        name='slam',
        output='screen',
        parameters=[{
            'use_sim_time': sim_time,
            'map_name': map_name,
        }],
    ))

    # 3) Web driving app.
    actions.append(Node(
        package='the_robot',
        executable='web_teleop',
        name='web_teleop',
        output='screen',
        parameters=[{'use_sim_time': sim_time}],
    ))

    # 4) Optional RViz with slam_toolbox's default config.
    if rviz:
        rviz_cfg = os.path.join(
            get_package_share_directory('slam_toolbox'),
            'config', 'slam_toolbox_default.rviz',
        )
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg] if os.path.isfile(rviz_cfg) else [],
            parameters=[{'use_sim_time': sim_time}],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo (sim) clock. Set false on a real robot.',
        ),
        DeclareLaunchArgument(
            'sim', default_value='true',
            description="Bring up Gazebo (sim.launch.py). Set false to use the "
                        "real base bringup (robot_bringup.launch.py) instead.",
        ),
        DeclareLaunchArgument(
            'map_name', default_value='hospital',
            description='Base filename under maps/ for the saved/continued map '
                        '(<map_name>.pgm/.yaml and <map_name>_serial.posegraph).',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also open RViz with the slam_toolbox default view.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
