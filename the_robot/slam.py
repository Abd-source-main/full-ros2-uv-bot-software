"""SLAM node for the_robot.

Launches slam_toolbox (online async mapping) using a terminal command run
through subprocess, then saves the generated map to pgm/yaml files when the
node is shut down (Ctrl+C).
"""

import os
import subprocess

import rclpy
from rclpy.node import Node


class my_node(Node):
    """Node to run slam_toolbox and save the map as pgm/yaml."""

    def __init__(self):
        super().__init__('slam')
        self.get_logger().info('Running slam_toolbox........')

        # Where the saved map (pgm + yaml) will be written. ~ is expanded here
        # because map_saver_cli does NOT expand it inside 'map:=...' style args.
        maps_dir = os.path.expanduser('~/ros2_ws/src/the_robot/maps')
        os.makedirs(maps_dir, exist_ok=True)
        self.map_path = os.path.join(maps_dir, 'hospital')

        # Run slam_toolbox using subprocess (inherits os.environ). use_sim_time
        # is true because we are mapping inside the Gazebo simulation.
        self.slam_process = subprocess.Popen([
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            'use_sim_time:=true'
        ])

    def save_map(self):
        """Save the current map to pgm/yaml using map_saver_cli."""
        self.get_logger().info(f'Saving map to {self.map_path}.pgm / .yaml')
        # map_saver_cli writes <path>.pgm and <path>.yaml from the /map topic.
        subprocess.run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '--ros-args', '-p', 'use_sim_time:=true',
            '-f', self.map_path
        ])

    def destroy_node(self):
        # Save the map before tearing slam_toolbox down.
        self.save_map()
        self.slam_process.terminate()
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
