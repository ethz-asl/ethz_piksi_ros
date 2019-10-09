#include "piksi_multi_cpp/device.h"

#include <ros/console.h>
#include <memory>
#include <vector>
#include "piksi_multi_cpp/device_usb.h"

namespace piksi_multi_cpp {

Device::Device(const Identifier& id) : id_(id) {}

int32_t Device::read_redirect(uint8_t* buff, uint32_t n, void* context) {
  if (!context) {
    ROS_ERROR_STREAM("No context set.");
    return 0;
  }
  // Cast context to instance.
  Device* instance = static_cast<Device*>(context);
  // Execute instance's read function.
  return instance->read(buff, n);
}

std::shared_ptr<Device> Device::create(Type type, const Identifier& id) {
  if (type == Type::kUSB) {
    return std::shared_ptr<Device>(new DeviceUSB(id));
  } else {
    return nullptr;
  }
}

std::vector<std::shared_ptr<Device>> Device::createAllDevices() {
  std::vector<std::shared_ptr<Device>> devices;

  // Create all USB devices.
  Identifiers usb_ids = DeviceUSB::getAllIdentifiers();
  for (auto id : usb_ids) {
    auto dev = create(Type::kUSB, id);
    if (dev.get()) devices.push_back(dev);
  }

  ROS_WARN_COND(devices.empty(), "No device created.");
  return devices;
}

}  // namespace piksi_multi_cpp
