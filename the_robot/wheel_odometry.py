"""Wheel odometry node: publishes /odom and broadcasts the odom -> base_footprint TF.

This is the localization source used when there is no LiDAR/AMCL (see
nav2.launch.py localization:=none). It completes the TF chain Nav2 needs:

    map -> odom            (static transform, from nav2.launch.py)
    odom -> base_footprint (THIS node, dynamic)
    base_footprint -> ...  (robot_state_publisher, from the URDF)

Two input modes (parameter `source`):

  * 'ticks'   -- real hardware. Subscribes to `wheel_ticks`
                 (std_msgs/Int32MultiArray, data = [left_ticks, right_ticks]),
                 the cumulative encoder counts from your microcontroller, and
                 integrates true differential-drive odometry. Use this on the
                 real robot -- it is the only mode that reflects reality.

  * 'cmd_vel' -- open-loop fallback for bench testing WITHOUT encoders.
                 Integrates the commanded /cmd_vel Twist. It drifts quickly
                 (it assumes the robot perfectly executes every command) and is
                 NOT suitable for real navigation -- it just lets the TF tree
                 come up so you can test the rest of the stack.

Geometry parameters (MEASURE THESE on your robot):
    wheel_radius      metres, drive-wheel radius
    wheel_separation  metres, distance between the two drive wheels
    ticks_per_rev     encoder counts per full wheel revolution (ticks mode only)
"""

import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


class WheelOdometry(Node):
    """Integrate wheel motion into /odom + the odom -> base_footprint transform."""

    def __init__(self):
        super().__init__('wheel_odometry')

        # --- robot geometry (measure these!) ---
        self.declare_parameter('wheel_radius', 0.033)      # m
        self.declare_parameter('wheel_separation', 0.16)   # m
        self.declare_parameter('ticks_per_rev', 4096)      # counts / wheel rev

        # --- frames / topics / behaviour ---
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('source', 'cmd_vel')        # 'ticks' | 'cmd_vel'
        self.declare_parameter('publish_rate', 50.0)       # Hz (cmd_vel mode)

        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.tpr = self.get_parameter('ticks_per_rev').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.source = self.get_parameter('source').value

        # Integrated pose (in the odom frame).
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)

        self.last_t = self.get_clock().now()

        if self.source == 'ticks':
            self.last_l = None
            self.last_r = None
            self.create_subscription(
                Int32MultiArray, 'wheel_ticks', self.on_ticks, 10)
            self.get_logger().info(
                'wheel_odometry: ticks mode (subscribing to wheel_ticks)')
        else:
            # Open-loop integration of the last commanded velocity.
            self.cmd = Twist()
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
            rate = self.get_parameter('publish_rate').value
            self.create_timer(1.0 / rate, self.on_timer)
            self.get_logger().warn(
                'wheel_odometry: cmd_vel (open-loop) mode -- drifts, testing only')

    # --- ticks mode: true encoder-based odometry ---------------------------
    def on_ticks(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn('wheel_ticks needs [left, right]; ignoring')
            return
        left, right = msg.data[0], msg.data[1]
        if self.last_l is None:
            self.last_l, self.last_r = left, right
            return

        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds / 1e9
        if dt <= 0.0:
            return

        m_per_tick = (2.0 * math.pi * self.R) / self.tpr
        d_l = (left - self.last_l) * m_per_tick
        d_r = (right - self.last_r) * m_per_tick
        self.last_l, self.last_r = left, right

        d_center = (d_l + d_r) / 2.0
        d_th = (d_r - d_l) / self.L
        self._integrate_and_publish(d_center, d_th, dt, now)

    # --- cmd_vel mode: open-loop integration -------------------------------
    def on_cmd(self, msg):
        self.cmd = msg

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds / 1e9
        if dt <= 0.0:
            return
        d_center = self.cmd.linear.x * dt
        d_th = self.cmd.angular.z * dt
        self._integrate_and_publish(d_center, d_th, dt, now)

    # --- shared: integrate a motion increment and publish ------------------
    def _integrate_and_publish(self, d_center, d_th, dt, now):
        # Midpoint (2nd-order) integration of the differential-drive pose.
        self.x += d_center * math.cos(self.th + d_th / 2.0)
        self.y += d_center * math.sin(self.th + d_th / 2.0)
        self.th += d_th
        self.last_t = now

        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)
        stamp = now.to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_th / dt
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C (SIGINT) or a launch/Nav2 SIGTERM -- shut down quietly.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
