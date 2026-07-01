"""Custom keyboard teleop: W/A/S/D to drive, E to stop, publishes to /cmd_vel."""

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

BINDINGS = {
    'w': (1.0, 0.0),   # forward
    's': (-1.0, 0.0),  # backward
    'a': (0.0, 1.0),   # turn left
    'd': (0.0, -1.0),  # turn right
    'e': (0.0, 0.0),   # stop
}

HELP = """
Custom WASD teleop
------------------
   w        : forward
a  s  d     : left / back / right
   e        : stop

q / Ctrl-C  : quit
"""


class TeleopWASD(Node):
    """Reads single keypresses and publishes Twist commands."""

    def __init__(self):
        super().__init__('teleop_wasd')

        self.declare_parameter('linear_speed', 0.22)   # m/s  (waffle max ~0.26)
        self.declare_parameter('angular_speed', 0.5)   # rad/s
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear * self.linear_speed
        twist.angular.z = angular * self.angular_speed
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

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings).lower()
            if key in BINDINGS:
                linear, angular = BINDINGS[key]
                node.publish(linear, angular)
            elif key == 'q' or key == '\x03':  # q or Ctrl-C
                break
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'teleop error: {exc}')
    finally:
        node.publish(0.0, 0.0)  # stop the robot on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
