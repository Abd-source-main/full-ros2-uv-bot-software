"""Bring up the full simulation + Nav2 stack on the hospital map.

Launches Gazebo with the robot (and its TF tree via robot_state_publisher),
then map_server, AMCL, the Nav2 navigation servers and RViz.

Localization: by default AMCL is seeded with the INITIAL_POSE defined below, so
the robot knows where it is on the map automatically at launch (no "2D Pose
Estimate" needed). Edit INITIAL_POSE to match where the robot actually starts,
or set 'set' to False to fall back to seeding it manually in RViz.

This is the single entry point for navigation: `ros2 launch the_robot
nav2.launch.py`. (sim.launch.py is available separately for sim-only work.)
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# ===========================================================================
# EDIT ME — the robot's known starting pose on the map.
#
#   'set'  : True  -> AMCL localizes here automatically at launch.
#            False -> localize manually with RViz's "2D Pose Estimate".
#   'x','y': metres in the map frame.
#   'yaw'  : heading in radians (0 = facing +x, pi/2 = facing +y).
#
# The spawn pose in sim.launch.py / spawn_robot.py is (0, 0, yaw 0), so the
# defaults below match a fresh simulation. Change these if you move the robot's
# start point (or for a real robot, set them to its known dock/home pose).
# ===========================================================================
INITIAL_POSE = {
    'set': True,
    'x': 0.0,
    'y': 0.0,
    'yaw': 0.0,
}


def _build_params_file(base_params_path):
    """Return a params file path with INITIAL_POSE injected into AMCL.

    Reads the base Nav2 params YAML, adds set_initial_pose / initial_pose to the
    amcl node, and writes the result to a temporary file. RewrittenYaml cannot be
    used here because it only overrides keys that already exist, and the stock
    waffle.yaml has no initial-pose keys.
    """
    if not INITIAL_POSE['set']:
        # Manual mode: use the params unchanged.
        return base_params_path

    with open(base_params_path) as f:
        params = yaml.safe_load(f)

    amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {
        'x': float(INITIAL_POSE['x']),
        'y': float(INITIAL_POSE['y']),
        'z': 0.0,
        'yaw': float(INITIAL_POSE['yaw']),
    }

    tmp = tempfile.NamedTemporaryFile(
        mode='w', prefix='nav2_params_', suffix='.yaml', delete=False
    )
    yaml.safe_dump(params, tmp)
    tmp.close()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    """Build the launch actions (run as an OpaqueFunction so we can resolve the
    params_file path and generate the AMCL initial-pose params at launch time)."""
    the_robot_dir = get_package_share_directory('the_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    # Resolve the base params path now, then inject the initial pose.
    base_params = LaunchConfiguration('params_file').perform(context)
    params_file = _build_params_file(base_params)

    # Only spin up the Gazebo simulation when running in sim mode. On a real
    # robot (use_sim_time:=false) the hardware drivers provide the TF tree and
    # sensor topics, so we must NOT launch Gazebo.
    sim_enabled = use_sim_time.perform(context).lower() in ('true', '1')

    actions = []

    if sim_enabled:
        # Gazebo + robot spawn + robot_state_publisher (TF tree). Without the TF
        # tree AMCL cannot transform the laser scan into the robot frame and
        # never localizes, so this must come up alongside Nav2.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(the_robot_dir, 'launch', 'sim.launch.py')
            ),
        ))

    # map_server + AMCL + planner + controller + bt_navigator + lifecycle managers.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    # RViz with the Nav2 view (initial-pose + goal tools).
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    actions += [nav2_bringup, rviz]
    return actions


def generate_launch_description():
    """Generate the launch description."""
    the_robot_dir = get_package_share_directory('the_robot')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')

    # Default paths: hospital map shipped in this package, official waffle params.
    default_map = os.path.join(the_robot_dir, 'maps', 'hospital.yaml')
    default_params = os.path.join(
        tb3_nav2_dir, 'param', 'humble', 'waffle.yaml'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true.',
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map YAML file to load.',
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the base Nav2 parameters YAML file '
                    '(the AMCL initial pose is injected on top of it).',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start the Nav2 lifecycle nodes.',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_autostart,
        OpaqueFunction(function=launch_setup),
    ])
