from typing import Any, List, Mapping, Tuple

import dbus
from enum import Enum
from xml.etree import ElementTree as ET


class BatteryState(Enum):
    Unknown = 0
    Charging = 1
    Discharging = 2
    Empty = 3
    Fully_charged = 4
    Pending_charge = 5
    Pending_discharge = 6


class DbusTypeMapping:
    b = bool
    d = float
    i = int
    s = str
    t = int
    u = int
    x = int


"""
Indexes are dbus values
Values are matching include/linux/power_supply.h
"""
dbus_battery_health_mapping = [0, 1, 2, 0, 4, 3, 3]
dbus_battery_level_mapping = [0, 0, None, 2, 1, None, 3, 4, 5]
dbus_battery_status_mapping = [0, 1, 2, 0, 4, 3, 3]
dbus_battery_technology_mapping = [0, 2, 3, 4, 0, 5, 1]


class UPowerBase(object):
    def __init__(self, upower_interface: str = "org.freedesktop.UPower", upower_path: str = "/org/freedesktop/UPower"):
        self.upower_interface_name = upower_interface
        self.upower_name = "org.freedesktop.UPower"
        self.upower_path = upower_path

        self.bus = dbus.SystemBus()
        self.bus_obj = self.bus.get_object(self.upower_name, self.upower_path)

        self.upower_interface = dbus.Interface(self.bus_obj, self.upower_interface_name)

        self.property_interface = dbus.Interface(self.bus_obj, dbus.PROPERTIES_IFACE)
        self.properties = dict()
        self._nodes = dict()

        introspection_interface = dbus.Interface(self.bus_obj, dbus.INTROSPECTABLE_IFACE)
        introspection_result = introspection_interface.Introspect()
        root = ET.fromstring(introspection_result)
        for child in root.findall("interface"):
            child_props = child.findall("property")
            if child_props:
                for prop in child_props:
                    if "read" in prop.get("access"):
                        self.properties[prop.get("name")] = (
                            child.get("name"),
                            prop.get("name"),
                            getattr(DbusTypeMapping, prop.get("type")),
                        )

        for child in root.findall("node"):
            node_name = child.get("name")
            if self.upower_path.endswith("devices"):
                self._nodes[node_name] = UPowerDevice(upower_path=self.upower_path + "/" + node_name)
            elif self.upower_path.endswith("Wakeups"):
                self._nodes[node_name] = UPowerBase(
                    upower_interface="org.freedesktop.UPower.Wakeups", upower_path=self.upower_path + "/" + node_name
                )
            else:
                self._nodes[node_name] = UPowerBase(upower_path=self.upower_path + "/" + node_name)
            setattr(self, node_name, self._nodes[node_name])

    def __getitem__(self, item):
        if item not in self.properties:
            raise KeyError("{} not in self.properties".format(item))
        else:
            (interface, prop, mapping) = self.properties[item]
            return mapping(self.property_interface.Get(interface, prop))

    def __iter__(self):
        return self.properties.__iter__()


class UPowerManager(UPowerBase):
    def __init__(self, upower_interface: str = "org.freedesktop.UPower", upower_path: str = "/org/freedesktop/UPower"):
        super(UPowerManager, self).__init__(upower_interface, upower_path)

        self._devices = dict()
        if hasattr(self, "devices"):
            for dev in self.devices._nodes.values():
                self._devices[dev.upower_path] = dev

    def get_power_devices(self) -> List[UPowerBase]:
        devices = map(str, self.upower_interface.EnumerateDevices())
        return [self._devices[dev] for dev in devices]

    def get_display_device(self) -> UPowerBase:
        disp_dev = str(self.upower_interface.GetDisplayDevice())
        return self._devices[disp_dev]

    def get_critical_action(self) -> str:
        return self.upower_interface.GetCriticalAction()

    def get_device_percentage(self, battery) -> float:
        return self._devices[battery]["Percentage"]

    def get_full_device_information(self, battery: str) -> Mapping[str, Any]:
        return self._devices[battery].get_full_device_information()

    def is_lid_present(self) -> bool:
        return self["LidIsPresent"]

    def is_lid_closed(self) -> bool:
        return self["LidIsClosed"]

    def on_battery(self) -> bool:
        return self["OnBattery"]

    def has_wakeup_capabilities(self) -> bool:
        if not hasattr(self, "Wakeups"):
            return False
        return bool(self.Wakeups["HasCapability"])

    def get_wakeups_data(self) -> List[Tuple[bool, int, float, str, str]]:
        if not hasattr(self, "Wakeups"):
            return list()

        return self.Wakeups.upower_interface.GetData()

    def get_wakeups_total(self) -> int:
        if not hasattr(self, "Wakeups"):
            return 0

        return self.Wakeups.upower_interface.GetTotal()

    def is_charging(self, battery: str) -> bool:
        state = int(self._devices[battery]["State"])
        return state == BatteryState.Charging.value

    def get_state(self, battery: str) -> str:
        state = int(self._devices[battery]["State"])
        return BatteryState(state).name


class UPowerDevice(UPowerBase):
    def __init__(
        self,
        upower_interface: str = "org.freedesktop.UPower.Device",
        upower_path: str = "/org/freedesktop/UPower/devices/battery_BAT0",
    ):
        super(UPowerDevice, self).__init__(upower_interface, upower_path)

    def get_full_device_information(self) -> Mapping[str, Any]:
        information_table = {}
        for k in self.properties:
            information_table[k] = self[k]
        return information_table
