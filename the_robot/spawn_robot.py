"""Node that spawns the TurtleBot3 Waffle model into a running Gazebo."""

import os
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class spawn_robot(Node):
    """Node to spawn the waffle robot into Gazebo."""

    def __init__(self):
        super().__init__('spawn_robot')

        # Spawn pose (x, y, z)
        x, y, z = 0.0, 0.0, 0.1

        # Waffle SDF shipped with turtlebot3_gazebo
        model_sdf = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models', 'turtlebot3_waffle', 'model.sdf'
        )

        self.get_logger().info(
            f'Spawning waffle at x={x}, y={y}, z={z} from {model_sdf}'
        )

        # Use gazebo_ros spawn_entity.py against the running gzserver
        self.spawn_process = subprocess.Popen([
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'waffle',
            '-file', model_sdf,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
        ])


def main(args=None):
    rclpy.init(args=args)
    node = spawn_robot()
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
