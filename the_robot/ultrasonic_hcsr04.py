"""HC-SR04 ultrasonic driver using gpiozero (no external package / wiringPi).

Reads an HC-SR04 with gpiozero.DistanceSensor -- the same library motor_driver
uses for the motors -- and publishes sensor_msgs/Range on
/HCSR04_ultrasonic/distance. It is a drop-in replacement for the lib_hcsr04_ros
C++ driver, so collision_guard and guarded_base.launch.py work unchanged.

REAL ROBOT ONLY: needs the Raspberry Pi GPIO. Install once on the Pi:
    sudo apt install python3-gpiozero python3-lgpio

Pins are BCM/GPIO numbers (gpiozero convention). Wiring for this robot:
    TRIG -> GPIO16  (physical pin 36)
    ECHO -> GPIO13  (physical pin 33)  *** through a voltage divider ***
The HC-SR04 ECHO line is 5V; the Pi GPIO is 3.3V-only. Drop it with a divider
(e.g. 1k in series + 2k to ground) or you can damage the pin.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range

from gpiozero import DistanceSensor


class UltrasonicHCSR04(Node):
    """Publish HC-SR04 distance as sensor_msgs/Range via gpiozero."""

    def __init__(self):
        super().__init__('ultrasonic_hcsr04')

        # BCM/GPIO pin numbers. TRIG=GPIO16 (phys 36), ECHO=GPIO13 (phys 33).
        self.declare_parameter('trigger_pin', 16)
        self.declare_parameter('echo_pin', 13)

        # Range message metadata / sensor limits (metres, degrees).
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('minimum_range', 0.03)
        self.declare_parameter('maximum_range', 4.0)
        self.declare_parameter('field_of_view_deg', 15.0)

        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('range_topic', '/HCSR04_ultrasonic/distance')

        trig = self.get_parameter('trigger_pin').value
        echo = self.get_parameter('echo_pin').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('minimum_range').value
        self.max_range = self.get_parameter('maximum_range').value
        fov_deg = self.get_parameter('field_of_view_deg').value
        rate = self.get_parameter('publish_rate').value
        topic = self.get_parameter('range_topic').value

        self.fov_rad = fov_deg * 3.14159265 / 180.0

        # gpiozero handles the trigger pulse + echo timing for us. max_distance
        # caps and scales the reading; readings beyond it report as max_distance.
        self.sensor = DistanceSensor(
            echo=echo, trigger=trig, max_distance=self.max_range)

        self.pub = self.create_publisher(Range, topic, 10)
        self.create_timer(1.0 / rate, self.on_tick)

        self.get_logger().info(
            f'ultrasonic_hcsr04 up: TRIG=GPIO{trig}, ECHO=GPIO{echo}, '
            f'max={self.max_range} m, topic="{topic}".')

    def on_tick(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov_rad
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = float(self.sensor.distance)  # gpiozero: metres
        self.pub.publish(msg)

    def stop(self):
        self.sensor.close()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicHCSR04()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
