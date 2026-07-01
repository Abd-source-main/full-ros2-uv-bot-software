"""Publish a single Nav2 goal (PoseStamped on /goal_pose) from ROS parameters.

The goal is described in the map frame via parameters:

    x         # required, metres in map frame
    y         # required, metres in map frame
    yaw       # optional, radians heading (default 0.0)
    frame_id  # optional (default "map")

Usage:
    ros2 run the_robot nav2_goal --ros-args -p x:=2.0 -p y:=1.0
    ros2 run the_robot nav2_goal --ros-args -p x:=2.0 -p y:=1.0 -p yaw:=1.57

Run it while the Nav2 stack is up (ros2 launch the_robot nav2.launch.py) and the
robot is localized. The node waits for Nav2 to subscribe, publishes the goal,
then exits.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class Nav2GoalPublisher(Node):
    """Read a goal from parameters and publish it once to /goal_pose."""

    def __init__(self):
        super().__init__('nav2_goal')

        # Default is 0, Which is our docking station.
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')

        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value
        frame_id = self.get_parameter('frame_id').value

        
        self._goal_msg = self._build_msg(x, y, yaw, frame_id)

        # Match the QoS bt_navigator uses for /goal_pose (reliable, volatile). Quality of Service
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE
        self._pub = self.create_publisher(PoseStamped, '/goal_pose', qos)

        self.get_logger().info(
            f'Goal: x={x}, y={y}, yaw={yaw}, frame={frame_id}'
        )
        self._attempts = 0
        self._queue_ticks = 0  # ticks since a subscriber appeared
        # Publish on a timer so we can wait for Nav2 to subscribe first.
        self._timer = self.create_timer(0.1, self._try_publish)

    def _build_msg(self, x, y, yaw, frame_id):
        msg = PoseStamped()
        msg.header.frame_id = str(frame_id)
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        # Heading: rotation about Z only (flat robot) -> quaternion.
        yaw = float(yaw)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def _try_publish(self):
        self._attempts += 1
        if self._pub.get_subscription_count() == 0:
            if self._attempts <= 100:  # wait up to ~10 s for Nav2
                self.get_logger().info('Waiting for /goal_pose subscriber (Nav2)...')
                return
            self.get_logger().warn(
                'No subscriber on /goal_pose after 10s; publishing anyway.'
            )
        else:
            # Subscriber present: give it a few ticks to settle before publishing.
            self._queue_ticks += 1
            if self._queue_ticks < 25:
                self.get_logger().info('queuing data in /goal_pose')
                return

        self.get_logger().info(f'Attempts: {self._attempts}')
        self._goal_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._goal_msg)
        p = self._goal_msg.pose.position
        self.get_logger().info(
            f'Published Nav2 goal: x={p.x}, y={p.y}, z={p.z}, '
            f'frame={self._goal_msg.header.frame_id}'
        )
        self._timer.cancel()
        # Let the message flush, then shut down.
        self.create_timer(1.0, self._shutdown)
    def _shutdown(self):
        if rclpy.ok():
            self.get_logger().info('Shutting down nav2_goal node.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalPublisher()
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
