from upower_ros.upower import UPowerDevice, dbus_battery_status_mapping, dbus_battery_technology_mapping, BatteryState as BatteryStateEnum

import rospy
from sensor_msgs.msg import BatteryState

import socket


class BatteryNode(object):
    def __init__(self, battery_topic=None, topic_rate=None, upower_path=None, location=None):
        # type: (Union[str, None], Union[float, None], Union[str, None]) -> None
        """
        If parameter isn't provided. The ROS parameter server is checked (private params). Otherwise the default value
        is used.
        :param battery_topic: Topic to publish on (Default: "battery_state")
        :type battery_topic: str
        :param topic_rate: Rate of publishing (Default: 1.0)
        :type topic_rate: float
        :param upower_path: UPower path of the battery (Default: "/org/freedesktop/UPower/devices/battery_BAT0")
        :type upower_path: str
        :param upower_path: Location of the battery (Default: "")
        :type upower_path: str
        """
        if battery_topic is None:
            battery_topic = rospy.get_param("~battery_topic", "battery_state")
        if topic_rate is None:
            topic_rate = rospy.get_param("~topic_rate", 1.0)
        if upower_path is None:
            upower_path = rospy.get_param("~upower_path", "/org/freedesktop/UPower/devices/battery_BAT0")
        if location is None:
            location = rospy.get_param("~location", "")

        assert isinstance(location, str), "Location must be a string"

        self.battery = UPowerDevice(upower_path=upower_path)
        self.location = location
        self.pub = rospy.Publisher(battery_topic, BatteryState, queue_size=1)
        self.rate = rospy.Rate(topic_rate)
        self.last_master_check = rospy.get_time()

    def generate_msg(self):
        # type: () -> BatteryState
        """
        Generate a BatteryState msg with the current state of the battery
        :return: filled BatteryState msg
        :rtype: BatteryState
        """
        msg = BatteryState()
        msg.header.stamp.secs = self.battery["UpdateTime"]
        msg.voltage = self.battery["Voltage"]
        msg.current = self.battery["EnergyRate"]
        if int(self.battery["State"]) != BatteryStateEnum.Charging.value:
            msg.current = -msg.current
        msg.charge = self.battery["Energy"]
        msg.capacity = self.battery["EnergyFull"]
        msg.design_capacity = self.battery["EnergyFullDesign"]
        msg.percentage = self.battery["Percentage"]/100.0
        msg.power_supply_status = dbus_battery_status_mapping[self.battery["State"]]
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = dbus_battery_technology_mapping[self.battery["Technology"]]
        msg.present = self.battery["IsPresent"]
        msg.location = self.location
        msg.serial_number = self.battery["Serial"]
        return msg

    def loop(self, check_master=False):
        """
        Loop function to publish with a constant rate

        :param check_master: Check connection to master at 1Hz
        :return: return code to indicate connection to master was lost
        """

        while not rospy.is_shutdown():
            self.publish()
            if check_master and rospy.get_time() >= self.last_master_check + 1:
                self.last_master_check = rospy.get_time()
                try:
                    rospy.get_master().getPid()
                except socket.error:
                    rospy.logdebug("Connection to master is lost")
                    return 1  # This should result in a non-zero error code of the entire program
            self.rate.sleep()

        return 0

    def publish(self):
        """
        Publish function to be called. Can be used if multiple batteries are monitored in one node.
        """
        self.pub.publish(self.generate_msg())
