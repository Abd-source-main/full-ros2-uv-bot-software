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
from launch_ros.actions import Node


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


def _inject_range_layer(params):
    """Add a RangeSensorLayer fed by the HC-SR04 driver to the local costmap.

    The layer marks obstacles from the /HCSR04_ultrasonic/distance Range topic,
    catching near-field / low / glass obstacles the 2D LiDAR misses. It is
    appended to whatever plugins the base params already define, so it works
    regardless of the exact stock waffle.yaml layout.
    """
    local = (params.setdefault('local_costmap', {})
                   .setdefault('local_costmap', {})
                   .setdefault('ros__parameters', {}))
    plugins = list(local.get('plugins', []))
    if 'range_layer' not in plugins:
        plugins.append('range_layer')
    local['plugins'] = plugins
    local['range_layer'] = {
        'plugin': 'nav2_costmap_2d::RangeSensorLayer',
        'enabled': True,
        'topics': ['/HCSR04_ultrasonic/distance'],
        'input_sensor_type': 'ALL',
        'phi': 1.2,
        'inflate_cone': 1.0,
        'no_readings_timeout': 0.0,
        'clear_threshold': 0.2,
        'mark_threshold': 0.8,
        'clear_on_max_reading': True,
    }


def _build_params_file(base_params_path, add_range_layer=False,
                       inject_initial_pose=True):
    """Return a params file path with INITIAL_POSE (and optionally the HC-SR04
    RangeSensorLayer) injected.

    Reads the base Nav2 params YAML, applies the requested overrides, and writes
    the result to a temporary file. RewrittenYaml cannot be used here because it
    only overrides keys that already exist, and the stock waffle.yaml has neither
    the initial-pose keys nor a range layer.

    inject_initial_pose is False when AMCL is disabled (localization:=none) --
    there is no AMCL node to seed, so the initial pose is irrelevant.
    """
    set_pose = inject_initial_pose and INITIAL_POSE['set']
    if not set_pose and not add_range_layer:
        # Nothing to inject: use the params unchanged.
        return base_params_path

    with open(base_params_path) as f:
        params = yaml.safe_load(f)

    if set_pose:
        amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
        amcl['set_initial_pose'] = True
        amcl['initial_pose'] = {
            'x': float(INITIAL_POSE['x']),
            'y': float(INITIAL_POSE['y']),
            'z': 0.0,
            'yaw': float(INITIAL_POSE['yaw']),
        }

    if add_range_layer:
        _inject_range_layer(params)

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
    use_ultrasonic = LaunchConfiguration('use_ultrasonic')

    # The HC-SR04 driver is real-robot only (needs a Raspberry Pi + wiringPi), so
    # gate both the driver node and its costmap layer behind use_ultrasonic.
    ultrasonic_enabled = use_ultrasonic.perform(context).lower() in ('true', '1')

    # Localization mode: 'amcl' (laser scan matching, needs a LiDAR on /scan) or
    # 'none' (no laser -> skip AMCL, localize by wheel odometry with a static
    # map -> odom transform; drifts, but works without a scanner).
    localization = LaunchConfiguration('localization').perform(context).lower()
    use_amcl = localization != 'none'

    # Resolve the base params path now, then inject the initial pose (and the
    # ultrasonic RangeSensorLayer when enabled). The AMCL initial pose is only
    # meaningful when AMCL is actually running.
    base_params = LaunchConfiguration('params_file').perform(context)
    params_file = _build_params_file(
        base_params, add_range_layer=ultrasonic_enabled, inject_initial_pose=use_amcl)

    # Only spin up the Gazebo simulation when running in sim mode. On a real
    # robot (use_sim_time:=false) the hardware drivers provide the TF tree and
    # sensor topics, so we must NOT launch Gazebo.
    sim_enabled = use_sim_time.perform(context).lower() in ('true', '1')

    actions = []

    if ultrasonic_enabled:
        # Start the HC-SR04 driver so it publishes /HCSR04_ultrasonic/distance,
        # which the injected local-costmap range_layer consumes.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(the_robot_dir, 'launch', 'ultrasonic.launch.py')
            ),
        ))

    if sim_enabled:
        # Gazebo + robot spawn + robot_state_publisher (TF tree + odom via the
        # Gazebo diff-drive plugin).
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(the_robot_dir, 'launch', 'sim.launch.py')
            ),
        ))
    elif not use_amcl:
        # Real robot without a laser: bring up the TF tree + wheel odometry so
        # the odom -> base_footprint transform exists (the map -> odom half is
        # the static publisher added below). With AMCL we assume a real base
        # driver provides these instead, so only do it in the no-laser case.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(the_robot_dir, 'launch', 'robot_bringup.launch.py')
            ),
        ))

    if use_amcl:
        # Full Nav2: map_server + AMCL + planner + controller + bt_navigator +
        # lifecycle managers.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
            }.items(),
        ))
    else:
        # No AMCL. Replace it with a static map -> odom transform (identity) so
        # the robot localizes purely by odometry, and run navigation + a
        # standalone map_server (bringup_launch would have started AMCL, which is
        # useless without a /scan).
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
        actions.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
        ))
        actions.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server'],
            }],
        ))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
            }.items(),
        ))

    # RViz with the Nav2 view (initial-pose + goal tools).
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    ))

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
        'use_sim_time', default_value='true',
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
    declare_use_ultrasonic = DeclareLaunchArgument(
        'use_ultrasonic', default_value='false',
        description='Real robot only: start the HC-SR04 ultrasonic driver and '
                    'add its RangeSensorLayer to the local costmap. Requires a '
                    'Raspberry Pi with wiringPi; leave false in sim / on a laptop.',
    )
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='amcl', choices=['amcl', 'none'],
        description="Localization mode. 'amcl': laser scan matching (needs a "
                    "LiDAR publishing /scan). 'none': no laser -- skip AMCL, "
                    "localize by wheel odometry with a static map->odom transform "
                    "and bring up the base (robot_state_publisher + "
                    "wheel_odometry). Drifts over time; use only without a LiDAR.",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_autostart,
        declare_use_ultrasonic,
        declare_localization,
        OpaqueFunction(function=launch_setup),
    ])
