"""ROS 2 node publishing the battery state reported by UPower."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from upower_ros.upower import (
    BatteryState as BatteryStateEnum,
    UPowerDevice,
    dbus_battery_status_mapping,
    dbus_battery_technology_mapping,
)


class BatteryNode(Node):
    """Node publishing a sensor_msgs/BatteryState from a UPower device."""

    def __init__(self) -> None:
        """Declare parameters, connect to the battery and set up publishing.

        The following parameters are declared:

        - battery_topic: topic to publish on (default: "battery_state")
        - topic_rate: rate of publishing in Hz (default: 1.0)
        - upower_path: UPower path of the battery
          (default: "/org/freedesktop/UPower/devices/battery_BAT0")
        - location: location of the battery (default: "")
        """
        super().__init__("battery_node")

        battery_topic = self.declare_parameter("battery_topic", "battery_state").value
        topic_rate = self.declare_parameter("topic_rate", 1.0).value
        upower_path = self.declare_parameter("upower_path", "/org/freedesktop/UPower/devices/battery_BAT0").value
        location = self.declare_parameter("location", "").value

        self.battery = UPowerDevice(upower_path=upower_path)
        self.location = location
        self.pub = self.create_publisher(BatteryState, battery_topic, 1)
        self.timer = self.create_timer(1.0 / topic_rate, self.publish)

    def generate_msg(self) -> BatteryState:
        """Generate a BatteryState message with the current state of the battery.

        :return: filled BatteryState message
        """
        msg = BatteryState()
        msg.header.stamp.sec = int(self.battery["UpdateTime"])
        msg.voltage = self.battery["Voltage"]
        msg.current = self.battery["EnergyRate"]
        if int(self.battery["State"]) != BatteryStateEnum.Charging.value:
            msg.current = -msg.current
        msg.charge = self.battery["Energy"]
        msg.capacity = self.battery["EnergyFull"]
        msg.design_capacity = self.battery["EnergyFullDesign"]
        msg.percentage = self.battery["Percentage"] / 100.0
        msg.power_supply_status = dbus_battery_status_mapping[self.battery["State"]]
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = dbus_battery_technology_mapping[self.battery["Technology"]]
        msg.present = self.battery["IsPresent"]
        msg.location = self.location
        msg.serial_number = self.battery["Serial"]
        return msg

    def publish(self) -> None:
        """Publish the current battery state.

        Can be used if multiple batteries are monitored in one node.
        """
        self.pub.publish(self.generate_msg())


def main(args: list[str] | None = None) -> None:
    """Run the battery node.

    :param args: command line arguments passed to rclpy
    """
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
