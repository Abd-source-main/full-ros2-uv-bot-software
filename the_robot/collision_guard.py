"""Collision guard: stop the robot when the HC-SR04 sees something too close.

This node sits *between* the command sources (teleop / Nav2) and the motor
driver, acting as a safety gate:

    teleop_wasd / nav2  --/cmd_vel-->  collision_guard  --/cmd_vel_safe-->  motor_driver

It subscribes to the ultrasonic distance published by the HC-SR04 driver
(lib_hcsr04_ros -> sensor_msgs/Range on /HCSR04_ultrasonic/distance) and to the
incoming /cmd_vel. It republishes the command on /cmd_vel_safe at a steady rate,
but as soon as the measured range drops below `stop_distance` it *blocks forward
motion* and keeps publishing a stopped command, so the robot halts before it
crashes and stays stopped for as long as the obstacle is there.

Escape is still allowed: reverse (negative linear.x) and rotation pass through,
so you can back the robot out of the obstacle. A hysteresis gap (`clear_distance`)
keeps it from stuttering right at the threshold.

Fail-safe behaviour:
  * If /cmd_vel goes stale (`cmd_timeout`), it publishes zeros -- matches the
    motor_driver watchdog so a dead teleop doesn't leave a command latched.
  * If the range goes stale (`range_timeout`, e.g. the sensor died) it blocks
    forward motion too, since a silent sensor can't prove the path is clear.
    Set `stop_on_sensor_timeout:=false` to disable (e.g. bench testing with no
    sensor wired).

Wiring it in: remap the motor driver's input to the guarded topic, e.g.
    ros2 run the_robot motor_driver --ros-args -r cmd_vel:=cmd_vel_safe
(the provided guarded_base.launch.py does this for you).
"""

from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


class CollisionGuard(Node):
    """Gate /cmd_vel on the HC-SR04 range and republish a safe command."""

    def __init__(self):
        super().__init__('collision_guard')

        # --- distances (metres) ---------------------------------------------
        # Below stop_distance we block forward motion; we only unblock once the
        # range climbs back above clear_distance (hysteresis, must be > stop).
        self.declare_parameter('stop_distance', 0.25)
        self.declare_parameter('clear_distance', 0.35)

        # --- timing ----------------------------------------------------------
        self.declare_parameter('publish_rate', 20.0)   # Hz to republish command
        self.declare_parameter('cmd_timeout', 0.5)     # s, zero out stale cmd_vel
        self.declare_parameter('range_timeout', 1.0)   # s, treat sensor as dead
        self.declare_parameter('stop_on_sensor_timeout', True)

        # --- range filtering -------------------------------------------------
        # HC-SR04 sensors emit occasional spurious readings (missed echoes,
        # crosstalk). Median-filter the last N samples so a single bad reading
        # can't flip the block and chop up forward motion. Also drop readings
        # below min_valid_range as noise rather than a very-close obstacle.
        self.declare_parameter('range_filter_size', 5)   # median window length
        self.declare_parameter('min_valid_range', 0.02)  # m, below = noise

        # --- topics ----------------------------------------------------------
        self.declare_parameter('range_topic', '/HCSR04_ultrasonic/distance')

        self.stop_distance = self.get_parameter('stop_distance').value
        self.clear_distance = self.get_parameter('clear_distance').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.range_timeout = self.get_parameter('range_timeout').value
        self.stop_on_timeout = self.get_parameter('stop_on_sensor_timeout').value
        self.min_valid_range = self.get_parameter('min_valid_range').value
        filter_size = max(1, int(self.get_parameter('range_filter_size').value))
        range_topic = self.get_parameter('range_topic').value
        rate = self.get_parameter('publish_rate').value

        if self.clear_distance <= self.stop_distance:
            self.get_logger().warn(
                f'clear_distance ({self.clear_distance}) <= stop_distance '
                f'({self.stop_distance}); hysteresis disabled, may chatter.')

        self.last_cmd = Twist()
        self.last_cmd_t = None      # None => no command received yet
        self.last_range = None      # median-filtered range, None => no valid data
        self.last_range_t = None
        self.range_window = deque(maxlen=filter_size)
        self.blocked = False        # latched state for hysteresis

        self.pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        # Range from the HC-SR04 driver -- sensor QoS for best_effort/reliable
        # compatibility regardless of how the driver publishes it.
        self.create_subscription(
            Range, range_topic, self.on_range, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.on_tick)

        self.get_logger().info(
            f'collision_guard up: stop<{self.stop_distance} m, '
            f'clear>{self.clear_distance} m, range="{range_topic}", '
            f'out="cmd_vel_safe".')

    def on_cmd(self, msg):
        self.last_cmd = msg
        self.last_cmd_t = self.get_clock().now()

    def on_range(self, msg):
        # Timestamp every message so the staleness watchdog tracks the sensor
        # node being alive, independent of whether this sample was usable.
        self.last_range_t = self.get_clock().now()
        # Keep only physically-plausible readings; a missed echo (0.0) or sub-
        # minimum noise spike is not a real obstacle, so it must not enter the
        # filter and drag the median below the stop threshold.
        if msg.range >= self.min_valid_range:
            self.range_window.append(msg.range)
        self.last_range = self._median(self.range_window)

    @staticmethod
    def _median(values):
        """Median of the window, or None if empty (no valid readings yet)."""
        if not values:
            return None
        ordered = sorted(values)
        n = len(ordered)
        mid = n // 2
        if n % 2:
            return ordered[mid]
        return 0.5 * (ordered[mid - 1] + ordered[mid])

    def _age(self, t):
        """Seconds since timestamp `t`; large number if never set."""
        if t is None:
            return float('inf')
        return (self.get_clock().now() - t).nanoseconds / 1e9

    def _update_blocked(self):
        """Latch the blocked flag with hysteresis from the latest range."""
        range_stale = self._age(self.last_range_t) > self.range_timeout

        if range_stale:
            # Can't prove the path is clear -> block (unless disabled).
            self.blocked = self.stop_on_timeout
            return

        d = self.last_range
        if d is None:
            self.blocked = self.stop_on_timeout
            return

        # Ignore out-of-range zero/negative readings some drivers emit; treat
        # only a positive distance below the threshold as an obstacle.
        if 0.0 < d < self.stop_distance:
            self.blocked = True
        elif d >= self.clear_distance:
            self.blocked = False
        # In the [stop_distance, clear_distance) band we hold the current state.

    def on_tick(self):
        was_blocked = self.blocked
        self._update_blocked()

        out = Twist()

        # Stale command -> publish zeros (nothing to pass through).
        if self._age(self.last_cmd_t) <= self.cmd_timeout:
            out.linear.x = self.last_cmd.linear.x
            out.linear.y = self.last_cmd.linear.y
            out.linear.z = self.last_cmd.linear.z
            out.angular.x = self.last_cmd.angular.x
            out.angular.y = self.last_cmd.angular.y
            out.angular.z = self.last_cmd.angular.z

        if self.blocked and out.linear.x > 0.0:
            # Block forward drive; keep reverse + rotation so we can escape.
            out.linear.x = 0.0

        if self.blocked and not was_blocked:
            self.get_logger().warn(
                f'obstacle at {self.last_range} m -- blocking forward motion.')
        elif was_blocked and not self.blocked:
            self.get_logger().info('path clear -- resuming.')

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
