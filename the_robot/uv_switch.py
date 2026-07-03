"""UV lamp switch: drives one GPIO pin ON or OFF from a ROS parameter.

REAL ROBOT ONLY (needs the Raspberry Pi GPIO). One output pin controls the UV
disinfection lamp -- typically the IN pin of a relay/MOSFET board that switches
the lamp's supply. The pin state follows the `on` parameter, which DEFAULTS TO
OFF so the lamp never comes up energised by accident (UV-C is hazardous to skin
and eyes).

    # start with the lamp OFF (default)
    ros2 run the_robot uv_switch

    # start with the lamp ON
    ros2 run the_robot uv_switch --ros-args -p on:=true

    # many relay boards are active-LOW (pin low = relay closed); flip that:
    ros2 run the_robot uv_switch --ros-args -p pin:=23 -p active_high:=false -p on:=true

The node keeps spinning so you can also toggle it live without restarting:

    ros2 param set /uv_switch on true
    ros2 param set /uv_switch on false

For safety the lamp is switched OFF when the node shuts down (Ctrl-C, crash, or
systemd stop), so a dying node never leaves UV-C running unattended.

Uses gpiozero (same backend as motor_driver):
    sudo apt install python3-gpiozero python3-lgpio
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from gpiozero import OutputDevice


class UVSwitch(Node):
    """Drive a single GPIO output for the UV lamp from the `on` parameter."""

    def __init__(self):
        super().__init__('uv_switch')

        # --- configuration (ROS args) ---------------------------------------
        # BCM/GPIO number wired to the relay/MOSFET input for the UV lamp.
        self.declare_parameter('pin', 24)
        # Relay boards vary: active_high=True -> pin HIGH turns the lamp on;
        # active_high=False for the common active-LOW opto-isolated boards.
        self.declare_parameter('active_high', True)
        # The one signal: on -> lamp energised, off -> lamp dark. Default OFF.
        self.declare_parameter('on', False)

        pin = self.get_parameter('pin').value
        active_high = self.get_parameter('active_high').value
        start_on = self.get_parameter('on').value

        # initial_value maps through active_high, so the physical pin is driven
        # to the correct level for "on"/"off" from the very first instant --
        # no brief flicker to the wrong state on startup.
        self.lamp = OutputDevice(pin, active_high=active_high, initial_value=start_on)

        # Allow live toggling via `ros2 param set /uv_switch on true/false`.
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f'uv_switch up. pin={pin} active_high={active_high} '
            f'-> lamp {"ON" if start_on else "OFF"}')

    def _apply(self, on):
        """Drive the pin to match the requested on/off state and log the change."""
        if on:
            self.lamp.on()
        else:
            self.lamp.off()
        self.get_logger().info(f'UV lamp {"ON" if on else "OFF"}')

    def _on_set_params(self, params):
        """Handle runtime `on` changes; reject anything non-boolean."""
        for p in params:
            if p.name == 'on':
                if p.type_ != p.Type.BOOL:
                    return SetParametersResult(
                        successful=False, reason='on must be a boolean')
                self._apply(p.value)
        return SetParametersResult(successful=True)

    def off(self):
        """Force the lamp off (used on shutdown for safety)."""
        self.lamp.off()


def main(args=None):
    rclpy.init(args=args)
    node = UVSwitch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Never leave UV-C running if the node goes away.
        node.off()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
