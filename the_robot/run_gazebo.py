"""Starter node for the_robot."""

import os
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class my_node(Node):
    """Node to run the simulation."""

    def __init__(self):
        super().__init__('run_gazebo')
        self.get_logger().info('Running Gazebo........')

        # Resolve absolute paths (~ is NOT expanded inside 'world:=...')
        maps_dir = os.path.expanduser('~/ros2_ws/src/the_robot/maps')
        models_dir = os.path.join(maps_dir, 'models')
        world_file = os.path.join(maps_dir, 'worlds', 'hospital.world')

        # Make the world's included models findable by Gazebo
        os.environ['GAZEBO_MODEL_PATH'] = (
            models_dir + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')
        )

        # Run the Gazebo simulation using subprocess (inherits os.environ)
        self.gazebo_process = subprocess.Popen([
            'ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
            f'world:={world_file}'
        ])

    def destroy_node(self):
        self.gazebo_process.terminate()
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
