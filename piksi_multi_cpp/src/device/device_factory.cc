#include <ros/console.h>
#include "piksi_multi_cpp/device/device.h"
#include "piksi_multi_cpp/device/device_factory.h"
#include "piksi_multi_cpp/device/device_usb.h"

namespace piksi_multi_cpp {

DevicePtr DeviceFactory::createByDeviceTypeAndSerialNumber(
    Device::DeviceType type, const SerialNumber& sn) {
  if (type == Device::DeviceType::kUSB) {
    return DevicePtr(new DeviceUSB(sn));
  } else {
    return nullptr;
  }
}

std::vector<DevicePtr> DeviceFactory::createAllDevicesByAutodiscovery() {
  std::vector<DevicePtr> devices;

  // Create all USB devices.
  SerialNumbers sns = DeviceUSB::discoverAllSerialNumbers();
  for (auto sn : sns) {
    auto dev = createByDeviceTypeAndSerialNumber(Device::DeviceType::kUSB, sn);
    if (dev.get()) devices.push_back(dev);
  }

  ROS_WARN_COND(devices.empty(), "No device created.");
  return devices;
}

}  // namespace piksi_multi_cpp
