#include "piksi_multi_cpp/device/device_usb.h"

#include <piksi_multi_cpp/device/device_tcp.h>
#include <ros/console.h>
#include <memory>
#include <vector>

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

}  // namespace piksi_multi_cpp
