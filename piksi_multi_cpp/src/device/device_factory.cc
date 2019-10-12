#include <ros/console.h>
#include "piksi_multi_cpp/device/device.h"
#include "piksi_multi_cpp/device/device_factory.h"
#include "piksi_multi_cpp/device/device_usb.h"

namespace piksi_multi_cpp {

Device::Ptr DeviceFactory::createByDeviceTypeAndSerialNumber(
    DeviceType type, const Identifier& id) {
  if (type == DeviceType::kUSB) {
    return Device::Ptr(new DeviceUSB(id));
  } else {
    return nullptr;
  }
}

std::vector<Device::Ptr> DeviceFactory::createAllDevicesByAutodiscovery() {
  std::vector<Device::Ptr> devices;

  // Create all USB devices.
  Identifiers ids = DeviceUSB::discoverAllSerialNumbers();
  for (auto id : ids) {
    auto dev = createByDeviceTypeAndSerialNumber(DeviceType::kUSB, id);
    if (dev.get()) devices.push_back(dev);
  }

  ROS_WARN_COND(devices.empty(), "No device created.");
  return devices;
}

}  // namespace piksi_multi_cpp
