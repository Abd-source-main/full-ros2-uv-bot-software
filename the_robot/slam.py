"""SLAM manager node for the_robot (drive-and-map workflow).

Launches slam_toolbox (online async mapping) as a subprocess, and on shutdown
saves the result in TWO forms:

  * a serialized pose graph (<map_name>_serial.posegraph / .data) via the
    slam_toolbox `serialize_map` service -- this is what lets a LATER launch
    CONTINUE the same map instead of starting blank.
  * an occupancy image (<map_name>.pgm / .yaml) via map_saver_cli -- consumed by
    web_goal.py and nav2.launch.py (map_server / AMCL).

On startup, if the serialized pose graph already exists, slam_toolbox is told to
load it (map_file_name + map_start_at_dock) so mapping resumes in the saved map.

Parameters
----------
use_sim_time : bool  (default True)   forwarded to slam_toolbox and map_saver.
map_name     : str   (default 'hospital')  base filename under maps/.
"""

import os
import subprocess

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


def _base_params_path():
    """Locate our slam_toolbox params: installed share dir, else the source tree,
    else the stock slam_toolbox online-async config."""
    try:
        share = get_package_share_directory('the_robot')
        cand = os.path.join(share, 'config', 'slam_toolbox.yaml')
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(os.path.dirname(here), 'config', 'slam_toolbox.yaml')
    if os.path.isfile(cand):
        return cand
    # Last resort: the stock params shipped with slam_toolbox.
    return os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config', 'mapper_params_online_async.yaml',
    )


class my_node(Node):
    """Runs slam_toolbox; serializes + saves the map on shutdown."""

    def __init__(self):
        super().__init__('slam')

        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('map_name', 'hospital')
        self.use_sim_time = bool(self.get_parameter('use_sim_time').value)
        map_name = self.get_parameter('map_name').value

        # Where maps are written. ~ is expanded here because map_saver_cli /
        # slam_toolbox do NOT expand it inside 'name:=...' style args.
        maps_dir = os.path.expanduser('~/ros2_ws/src/the_robot/maps')
        os.makedirs(maps_dir, exist_ok=True)
        self.map_path = os.path.join(maps_dir, map_name)               # pgm/yaml
        self.serial_path = os.path.join(maps_dir, f'{map_name}_serial')  # posegraph/data

        sim = 'true' if self.use_sim_time else 'false'

        # Build the slam params file. If a serialized graph from a previous run
        # exists, inject map_file_name + map_start_at_dock so we CONTINUE it.
        params_file = self._build_params_file()
        continuing = os.path.isfile(self.serial_path + '.posegraph')
        if continuing:
            self.get_logger().info(
                f'Continuing existing map: {self.serial_path}.posegraph')
        else:
            self.get_logger().info('No saved map found -- starting a fresh map.')

        self.get_logger().info('Running slam_toolbox (online async)...')
        # start_new_session=True puts slam_toolbox in its own process group so a
        # Ctrl+C to this node does NOT kill it before we get to serialize/save it.
        self.slam_process = subprocess.Popen(
            [
                'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                f'use_sim_time:={sim}',
                f'slam_params_file:={params_file}',
            ],
            start_new_session=True,
        )

    def _build_params_file(self):
        """Copy the base slam params, injecting the continue-map keys when a
        serialized graph exists. Returns the path to use as slam_params_file."""
        base = _base_params_path()
        with open(base) as f:
            params = yaml.safe_load(f)

        ros_params = params.setdefault('slam_toolbox', {}).setdefault(
            'ros__parameters', {})
        ros_params['mode'] = 'mapping'

        if os.path.isfile(self.serial_path + '.posegraph'):
            # slam_toolbox appends .posegraph/.data to map_file_name itself.
            ros_params['map_file_name'] = self.serial_path
            # Resume at the same dock/origin pose the map graph was started from
            # (the robot spawns at the world origin here, matching the dock).
            ros_params['map_start_at_dock'] = True

        out = os.path.expanduser('~/ros2_ws/src/the_robot/maps/.slam_params_active.yaml')
        with open(out, 'w') as f:
            yaml.safe_dump(params, f)
        return out

    def serialize_map(self):
        """Serialize the live pose graph so a future run can continue this map."""
        self.get_logger().info(
            f'Serializing pose graph to {self.serial_path}.posegraph/.data')
        try:
            subprocess.run(
                [
                    'ros2', 'service', 'call',
                    '/slam_toolbox/serialize_map',
                    'slam_toolbox/srv/SerializePoseGraph',
                    f"{{filename: '{self.serial_path}'}}",
                ],
                timeout=30,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('serialize_map timed out.')

    def save_map(self):
        """Save the occupancy grid to pgm/yaml (for web_goal / nav2 map_server)."""
        self.get_logger().info(f'Saving map image to {self.map_path}.pgm / .yaml')
        sim = 'true' if self.use_sim_time else 'false'
        try:
            subprocess.run(
                [
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '--ros-args', '-p', f'use_sim_time:={sim}',
                    '--', '-f', self.map_path,
                ],
                timeout=60,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver_cli timed out.')

    def destroy_node(self):
        # Persist the map BEFORE tearing slam_toolbox down (both need it alive).
        self.serialize_map()
        self.save_map()
        self.slam_process.terminate()
        try:
            self.slam_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.slam_process.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = my_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
