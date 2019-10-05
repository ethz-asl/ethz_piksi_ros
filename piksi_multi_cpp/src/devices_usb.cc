#include "piksi_multi_cpp/devices_usb.h"

#include <ros/console.h>

namespace piksi_multi_cpp {
DevicesUSB::DevicesUSB() {}

bool DevicesUSB::open() {
  // Initialize libusb.
  int init = libusb_init(nullptr);
  if (init < 0) {
    ROS_ERROR_STREAM("Cannot initialize libusb: " << init);
    return false;
  }

  // Discover all devices.
  libusb_device** list;
  ssize_t cnt = libusb_get_device_list(nullptr, &list);
  if (cnt < 0) {
    ROS_ERROR_STREAM("Invalid device list: " << cnt);
    return false;
  }

  // Identify Piksis on USB.
  std::vector<libusb_device*> piksis;
  for (ssize_t i = 0; i < cnt; ++i) {
    libusb_device* device = list[i];
    if (identifyPiksi(device)) {
      piksis.push_back(device);
    }
  }
  if (piksis.empty()) {
    ROS_INFO("No Piksi discovered on USB.");
    libusb_free_device_list(list, 1);
    return false;
  }
  ROS_INFO_STREAM("Discovered " << piksis.size() << " Piksi(s) on USB.");

  // Open devices.
  for (auto piksi : piksis) {
    libusb_device_handle* handle;
    int open = libusb_open(piksi, &handle);
    if (open < 0) {
      ROS_WARN_STREAM("Failed to open Piksi Multi on bus number "
                      << libusb_get_bus_number(piksi) << " with device address "
                      << libusb_get_device_address(piksi) << " Code: " << open);
      continue;
    }
    handles_.push_back(handle);
  }

  libusb_free_device_list(list, 1);

  return !handles_.empty();
}

bool DevicesUSB::read() {
  ROS_ERROR("Read USB not implemented.");
  return false;
}

bool DevicesUSB::close() {
  libusb_exit(NULL);
  return true;
}

bool DevicesUSB::identifyPiksi(libusb_device* dev) {
  if (!dev) return false;

  libusb_device_descriptor desc = {0};
  int err = libusb_get_device_descriptor(dev, &desc);
  if (err < 0) {
    ROS_WARN_STREAM("Could not get device descriptor: " << err);
    return false;
  }

  const uint16_t kIDVendor = 0x2E69;
  const uint16_t kIDProduct = 0x1001;

  if (desc.idVendor == kIDVendor && desc.idProduct == kIDProduct) {
    ROS_INFO_STREAM("Identified Piksi Multi on bus number "
                    << libusb_get_bus_number(dev) << " with device address "
                    << libusb_get_device_address(dev));
    return true;
  } else {
    return false;
  }
}

}  // namespace piksi_multi_cpp
