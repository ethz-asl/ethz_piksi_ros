#include "piksi_multi_cpp/device.h"

#include "piksi_multi_cpp/device_usb.h"

namespace piksi_multi_cpp {

std::shared_ptr<Device> Device::create(DeviceType type, const Identifier& id) {
  if (type == DeviceType::kUSB) {
    return std::shared_ptr<Device>(new DeviceUSB(id));
  } else {
    return nullptr;
  }
}

}  // namespace piksi_multi_cpp
