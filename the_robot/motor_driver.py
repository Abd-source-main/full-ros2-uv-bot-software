"""Motor driver: subscribes to /cmd_vel and drives an L298N dual H-bridge over GPIO.

Wiring (your board), an L298N-style driver with four direction pins:

    Left  motor : IN1 = GPIO 26 (OUT1)   IN2 = GPIO 17 (OUT2)
    Right motor : IN3 = GPIO 27 (OUT3)   IN4 = GPIO 22 (OUT4)

Each motor is controlled by a pair of pins. To spin forward we PWM the "forward"
pin while the "backward" pin is low; to reverse we swap them. This gives both
direction AND speed control, assuming the ENA/ENB enable pins on the L298N are
jumpered high (the usual default). If your board has no jumpers, tie ENA/ENB to
5V (or wire them to spare GPIOs and drive them high).

Uses gpiozero, which works across Raspberry Pi models (Pi 5 / Bookworm included).
Install with:  sudo apt install python3-gpiozero   (or  pip install gpiozero lgpio)

The differential-drive mixing turns the /cmd_vel Twist into per-wheel speeds:

    v_left  = linear.x - angular.z * wheel_separation / 2
    v_right = linear.x + angular.z * wheel_separation / 2

then normalises by `max_speed` to a -1.0 .. 1.0 duty cycle for each wheel.

A safety watchdog stops the motors if no /cmd_vel arrives within `cmd_timeout`
seconds, so the robot doesn't run away if teleop dies.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from gpiozero import Motor


class MotorDriver(Node):
    """Drive a two-motor L298N from /cmd_vel Twist messages."""

    def __init__(self):
        super().__init__('motor_driver')

        # --- pin assignment (BCM numbering), matches your wiring -------------
        self.declare_parameter('left_forward_pin', 26)    # OUT1
        self.declare_parameter('left_backward_pin', 17)   # OUT2
        self.declare_parameter('right_forward_pin', 27)   # OUT3
        self.declare_parameter('right_backward_pin', 22)  # OUT4

        # --- kinematics / behaviour -----------------------------------------
        self.declare_parameter('wheel_separation', 0.16)  # m, between drive wheels
        self.declare_parameter('max_speed', 0.22)         # m/s that maps to full duty
        self.declare_parameter('invert_left', False)      # flip if a wheel runs backwards
        self.declare_parameter('invert_right', False)
        self.declare_parameter('cmd_timeout', 0.5)        # s, stop if no cmd_vel

        lf = self.get_parameter('left_forward_pin').value
        lb = self.get_parameter('left_backward_pin').value
        rf = self.get_parameter('right_forward_pin').value
        rb = self.get_parameter('right_backward_pin').value

        self.sep = self.get_parameter('wheel_separation').value
        self.max_speed = self.get_parameter('max_speed').value
        self.inv_left = self.get_parameter('invert_left').value
        self.inv_right = self.get_parameter('invert_right').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.min_duty = self.get_parameter('min_duty').value

        # gpiozero.Motor(forward, backward) does the PWM + direction for us.
        self.left = Motor(forward=lf, backward=lb, pwm=True)
        self.right = Motor(forward=rf, backward=rb, pwm=True)
        self._set_pwm_frequency(self.left)
        self._set_pwm_frequency(self.right)

        # Remember the last duty written per motor so we don't re-arm the PWM
        # with an identical value on every 20 Hz message (each rewrite can glitch
        # the output). Only actual changes are pushed to the hardware.
        self._last_duty = {}

        self.last_cmd_t = self.get_clock().now()
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        # Watchdog: check periodically that commands are still fresh.
        self.create_timer(self.cmd_timeout / 2.0, self.on_watchdog)

        self.get_logger().info(
            f'motor_driver up. left(fwd={lf}, bwd={lb}) '
            f'right(fwd={rf}, bwd={rb}) max_speed={self.max_speed} m/s')

    def on_cmd(self, msg):
        self.last_cmd_t = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z
        v_left = v - w * self.sep / 2.0
        v_right = v + w * self.sep / 2.0

        self._drive(self.left, v_left, self.inv_left)
        self._drive(self.right, v_right, self.inv_right)

    def _set_pwm_frequency(self, motor):
        """Raise the PWM carrier frequency on both direction pins of a motor.

        gpiozero exposes the underlying PWMOutputDevice as forward_device /
        backward_device when pwm=True. Guarded so a non-PWM build or an API
        change just leaves the default frequency instead of crashing.
        """
        for attr in ('forward_device', 'backward_device'):
            dev = getattr(motor, attr, None)
            if dev is not None and hasattr(dev, 'frequency'):
                try:
                    dev.frequency = self.pwm_frequency
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'could not set PWM frequency: {exc}')

    def _drive(self, motor, wheel_speed, invert):
        """Set a motor to a -1..1 duty cycle from a wheel speed in m/s."""
        duty = wheel_speed / self.max_speed if self.max_speed > 0.0 else 0.0
        duty = max(-1.0, min(1.0, duty))
        if invert:
            duty = -duty

        # Stiction compensation: remap non-zero |duty| from (0,1] onto
        # [min_duty, 1] so even the slowest command produces enough torque to
        # turn the wheel instead of buzzing/stalling in place.
        if self.min_duty > 0.0 and duty != 0.0:
            mag = self.min_duty + (1.0 - self.min_duty) * abs(duty)
            duty = mag if duty > 0.0 else -mag

        # Only write when the value actually changes -- re-arming gpiozero's PWM
        # with an identical duty on every 20 Hz tick can glitch the output.
        if self._last_duty.get(id(motor)) == duty:
            return
        self._last_duty[id(motor)] = duty
        motor.value = duty  # gpiozero: >0 forward PWM, <0 backward PWM, 0 coast

    def on_watchdog(self):
        """Stop the motors if /cmd_vel has gone stale (teleop closed, etc.)."""
        age = (self.get_clock().now() - self.last_cmd_t).nanoseconds / 1e9
        if age > self.cmd_timeout:
            self.stop()

    def stop(self):
        self.left.stop()
        self.right.stop()
        # Forget the cached duties so the next real command is always written.
        self._last_duty[id(self.left)] = 0.0
        self._last_duty[id(self.right)] = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
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
