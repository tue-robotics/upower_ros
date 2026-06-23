"""Thin wrapper around the UPower D-Bus interface.

Most of the code is based on https://github.com/wogscpar/upower-python.
"""

from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING, Any
from xml.etree import ElementTree as ET

import dbus

if TYPE_CHECKING:
    from collections.abc import Iterator, Mapping


class BatteryState(Enum):
    """UPower battery state enumeration."""

    Unknown = 0
    Charging = 1
    Discharging = 2
    Empty = 3
    Fully_charged = 4
    Pending_charge = 5
    Pending_discharge = 6


class DbusTypeMapping:
    """Mapping of D-Bus type signatures to Python types."""

    b = bool
    d = float
    i = int
    s = str
    t = int
    u = int
    x = int


# Indexes are dbus values, values match include/linux/power_supply.h
dbus_battery_health_mapping = [0, 1, 2, 0, 4, 3, 3]
dbus_battery_level_mapping = [0, 0, None, 2, 1, None, 3, 4, 5]
dbus_battery_status_mapping = [0, 1, 2, 0, 4, 3, 3]
dbus_battery_technology_mapping = [0, 2, 3, 4, 0, 5, 1]


class UPowerBase:
    """Base class wrapping a UPower D-Bus object and its properties."""

    def __init__(
        self, upower_interface: str = "org.freedesktop.UPower", upower_path: str = "/org/freedesktop/UPower"
    ) -> None:
        """Connect to the system bus and introspect the UPower object.

        :param upower_interface: D-Bus interface name
        :param upower_path: D-Bus object path
        """
        self.upower_interface_name = upower_interface
        self.upower_name = "org.freedesktop.UPower"
        self.upower_path = upower_path

        self.bus = dbus.SystemBus()
        self.bus_obj = self.bus.get_object(self.upower_name, self.upower_path)

        self.upower_interface = dbus.Interface(self.bus_obj, self.upower_interface_name)

        self.property_interface = dbus.Interface(self.bus_obj, dbus.PROPERTIES_IFACE)
        self.properties: dict[str, tuple[str, str, Any]] = {}
        self._nodes: dict[str, UPowerBase] = {}

        introspection_interface = dbus.Interface(self.bus_obj, dbus.INTROSPECTABLE_IFACE)
        introspection_result = introspection_interface.Introspect()
        root = ET.fromstring(introspection_result)  # noqa: S314
        for child in root.findall("interface"):
            interface_name = child.get("name")
            if interface_name is None:
                continue
            for prop in child.findall("property"):
                access = prop.get("access")
                prop_name = prop.get("name")
                prop_type = prop.get("type")
                if access is None or prop_name is None or prop_type is None:
                    continue
                if "read" in access:
                    self.properties[prop_name] = (interface_name, prop_name, getattr(DbusTypeMapping, prop_type))

        for child in root.findall("node"):
            node_name = child.get("name")
            if node_name is None:
                continue
            if self.upower_path.endswith("devices"):
                self._nodes[node_name] = UPowerDevice(upower_path=self.upower_path + "/" + node_name)
            elif node_name == "Wakeups":
                self._nodes[node_name] = UPowerBase(
                    upower_interface="org.freedesktop.UPower.Wakeups", upower_path=self.upower_path + "/" + node_name
                )
            else:
                self._nodes[node_name] = UPowerBase(upower_path=self.upower_path + "/" + node_name)
            setattr(self, node_name, self._nodes[node_name])

    def __getitem__(self, item: str) -> Any:  # noqa: ANN401
        """Return the (typed) value of a UPower property.

        :param item: property name
        :return: property value cast to its Python type
        """
        if item not in self.properties:
            msg = f"{item} not in self.properties"
            raise KeyError(msg)
        interface, prop, mapping = self.properties[item]
        return mapping(self.property_interface.Get(interface, prop))

    def __iter__(self) -> Iterator[str]:
        """Iterate over the available property names."""
        return iter(self.properties)


class UPowerManager(UPowerBase):
    """UPower manager exposing the system-wide power devices and state."""

    def __init__(
        self, upower_interface: str = "org.freedesktop.UPower", upower_path: str = "/org/freedesktop/UPower"
    ) -> None:
        """Connect to the UPower manager and collect its power devices.

        :param upower_interface: D-Bus interface name
        :param upower_path: D-Bus object path
        """
        super().__init__(upower_interface, upower_path)

        self._devices: dict[str, UPowerDevice] = {}
        if hasattr(self, "devices"):
            for dev in self.devices._nodes.values():  # noqa: SLF001
                if isinstance(dev, UPowerDevice):
                    self._devices[dev.upower_path] = dev

    def get_power_devices(self) -> list[UPowerBase]:
        """Return all power devices known to UPower."""
        devices = map(str, self.upower_interface.EnumerateDevices())
        return [self._devices[dev] for dev in devices]

    def get_display_device(self) -> UPowerBase:
        """Return the composite display device."""
        disp_dev = str(self.upower_interface.GetDisplayDevice())
        return self._devices[disp_dev]

    def get_critical_action(self) -> str:
        """Return the action to take when the power is critically low."""
        return str(self.upower_interface.GetCriticalAction())

    def get_device_percentage(self, battery: str) -> float:
        """Return the charge percentage of the given device.

        :param battery: D-Bus path of the device
        :return: charge percentage
        """
        return float(self._devices[battery]["Percentage"])

    def get_full_device_information(self, battery: str) -> Mapping[str, Any]:
        """Return all properties of the given device.

        :param battery: D-Bus path of the device
        :return: mapping of property name to value
        """
        return self._devices[battery].get_full_device_information()

    def is_lid_present(self) -> bool:
        """Return whether a laptop lid is present."""
        return bool(self["LidIsPresent"])

    def is_lid_closed(self) -> bool:
        """Return whether the laptop lid is closed."""
        return bool(self["LidIsClosed"])

    def on_battery(self) -> bool:
        """Return whether the system is running on battery power."""
        return bool(self["OnBattery"])

    def has_wakeup_capabilities(self) -> bool:
        """Return whether UPower exposes wakeup statistics."""
        if not hasattr(self, "Wakeups"):
            return False
        return bool(self.Wakeups["HasCapability"])

    def get_wakeups_data(self) -> list[tuple[bool, int, float, str, str]]:
        """Return the per-process wakeup statistics."""
        if not hasattr(self, "Wakeups"):
            return []
        return list(self.Wakeups.upower_interface.GetData())

    def get_wakeups_total(self) -> int:
        """Return the total number of wakeups per second."""
        if not hasattr(self, "Wakeups"):
            return 0
        return int(self.Wakeups.upower_interface.GetTotal())

    def is_charging(self, battery: str) -> bool:
        """Return whether the given device is charging.

        :param battery: D-Bus path of the device
        :return: True if charging
        """
        state = int(self._devices[battery]["State"])
        return state == BatteryState.Charging.value

    def get_state(self, battery: str) -> str:
        """Return the textual state of the given device.

        :param battery: D-Bus path of the device
        :return: state name
        """
        state = int(self._devices[battery]["State"])
        return BatteryState(state).name


class UPowerDevice(UPowerBase):
    """UPower device, e.g. a battery."""

    def __init__(
        self,
        upower_interface: str = "org.freedesktop.UPower.Device",
        upower_path: str = "/org/freedesktop/UPower/devices/battery_BAT0",
    ) -> None:
        """Connect to a single UPower device.

        :param upower_interface: D-Bus interface name
        :param upower_path: D-Bus object path of the device
        """
        super().__init__(upower_interface, upower_path)

    def get_full_device_information(self) -> Mapping[str, Any]:
        """Return all readable properties of the device."""
        return {k: self[k] for k in self.properties}
