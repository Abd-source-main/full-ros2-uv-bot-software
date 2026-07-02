"""Custom keyboard teleop: tap W/A/S/D to accelerate, publishes to /cmd_vel.

Unlike a hold-to-move teleop, each keypress *increments* the target speed:

    w : go faster forward   (or brake if currently reversing)
    s : go faster backward  (or brake if currently going forward)
    a : turn left faster
    d : turn right faster
    e : stop (reset speed to zero)

The current target velocity is republished continuously by a background timer,
so the robot keeps moving after a single tap (and the motor_driver watchdog,
which stops the motors 0.5 s after the last /cmd_vel, stays happy).
"""

import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# (linear_delta, angular_delta) applied to the target on each keypress.
BINDINGS = {
    'w': (1.0, 0.0),   # accelerate forward
    's': (-1.0, 0.0),  # accelerate backward
    'a': (0.0, 1.0),   # turn left faster
    'd': (0.0, -1.0),  # turn right faster
}

HELP = """
Custom WASD teleop (tap to accelerate)
--------------------------------------
   w        : faster forward
a  s  d     : turn left / faster back / turn right
   e        : stop (reset speed)

q / Ctrl-C  : quit

Tip: tap repeatedly to build up speed; the robot keeps moving on its own.
"""


class TeleopWASD(Node):
    """Accumulates a target Twist from keypresses and republishes it."""

    def __init__(self):
        super().__init__('teleop_wasd')

        # Overall speed reduced: these are the *max* reachable speeds, and each
        # tap adds one step toward them.
        self.declare_parameter('max_linear', 0.12)     # m/s  (was 0.22)
        self.declare_parameter('max_angular', 0.4)     # rad/s (was 0.5)
        self.declare_parameter('linear_step', 0.03)    # m/s added per W/S tap
        self.declare_parameter('angular_step', 0.1)    # rad/s added per A/D tap
        self.declare_parameter('publish_rate', 20.0)   # Hz to republish target

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.linear_step = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value
        rate = self.get_parameter('publish_rate').value

        self.target_linear = 0.0
        self.target_angular = 0.0

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Republish the current target so the robot keeps moving between taps.
        self.create_timer(1.0 / rate, self._publish_target)

    def bump(self, d_linear, d_angular):
        """Adjust the target velocity by one step, clamped to the max."""
        self.target_linear += d_linear * self.linear_step
        self.target_angular += d_angular * self.angular_step
        self.target_linear = max(-self.max_linear, min(self.max_linear, self.target_linear))
        self.target_angular = max(-self.max_angular, min(self.max_angular, self.target_angular))

    def stop(self):
        self.target_linear = 0.0
        self.target_angular = 0.0

    def _publish_target(self):
        twist = Twist()
        twist.linear.x = self.target_linear
        twist.angular.z = self.target_angular
        self.publisher.publish(twist)


def get_key(settings):
    """Read a single keypress from stdin without waiting for Enter."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopWASD()

    # Spin in the background so the republish timer keeps firing while the main
    # loop blocks on keyboard input.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings).lower()
            if key in BINDINGS:
                node.bump(*BINDINGS[key])
                print(f'\rtarget: linear={node.target_linear:+.2f} m/s  '
                      f'angular={node.target_angular:+.2f} rad/s   ', end='', flush=True)
            elif key == 'e':
                node.stop()
                print('\rtarget: stopped                                   ',
                      end='', flush=True)
            elif key == 'q' or key == '\x03':  # q or Ctrl-C
                break
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'teleop error: {exc}')
    finally:
        node.stop()
        node._publish_target()  # send a final zero velocity
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
