#include "piksi_multi_cpp/device.h"

#include <memory>
#include <vector>
#include "piksi_multi_cpp/device_usb.h"

namespace piksi_multi_cpp {

int32_t Device::read_redirect(uint8_t* buff, uint32_t n, void* context) {
  // Cast context to instance.
  Device* instance = static_cast<Device*>(context);
  // Execute instance's read function.
  return instance->read(buff, n);
}

std::shared_ptr<Device> Device::create(DeviceType type, const Identifier& id) {
  if (type == DeviceType::kUSB) {
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
    devices.push_back(create(DeviceType::kUSB, id));
  }

  return devices;
}

}  // namespace piksi_multi_cpp
