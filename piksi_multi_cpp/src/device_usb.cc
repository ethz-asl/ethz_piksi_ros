#include "piksi_multi_cpp/device_usb.h"

#include <ros/console.h>

const int kInterfaceNumber = 0;

namespace piksi_multi_cpp {

DeviceUSB::DeviceUSB() : handle_(nullptr) {}

bool DeviceUSB::open() {
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
      ROS_INFO(
          "Failed to open Piksi Multi on bus number / address %03d.%03d: %d",
          libusb_get_bus_number(piksi), libusb_get_device_address(piksi), open);
      continue;
    }
    int detach_kernel = libusb_set_auto_detach_kernel_driver(handle, 1);
    if (detach_kernel < 0){
      ROS_WARN_STREAM("Cannot detach kernel driver: " << detach_kernel);
    }
    int claim = libusb_claim_interface(handle, kInterfaceNumber);
    if (claim < 0) {
      ROS_INFO_STREAM("Failed to claim device: " << claim);
      continue;
    }
    ROS_INFO_STREAM("New device "
                    << ":");
    printDeviceInfo(handle);
    handle_ = handle;
    break;
  }

  libusb_free_device_list(list, 1);
  return handle_ != nullptr;
}

bool DeviceUSB::read() {
  ROS_ERROR("Read USB not implemented.");
  return false;
}

bool DeviceUSB::close() {
  bool success = true;
  int release = libusb_release_interface(handle_, kInterfaceNumber);
  if (release < 0) {
    ROS_WARN_STREAM("Device was not released: " << release);
    success = false;
  }
  libusb_close(handle_);
  libusb_exit(nullptr);
  return success;
}

bool DeviceUSB::identifyPiksi(libusb_device* dev) {
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
    ROS_INFO("Identified Piksi on bus number / address %03d.%03d",
             libusb_get_bus_number(dev), libusb_get_device_address(dev));
    return true;
  } else {
    return false;
  }
}

void DeviceUSB::printDeviceInfo(libusb_device_handle* handle) {
  if (!handle) return;

  auto dev = libusb_get_device(handle);
  if (!dev) return;

  libusb_device_descriptor desc = {0};
  int err = libusb_get_device_descriptor(dev, &desc);
  if (err < 0) {
    ROS_WARN_STREAM("Could not get device descriptor: " << err);
    return;
  }

  if (desc.iManufacturer) {
    unsigned char string[256];
    int ret = libusb_get_string_descriptor_ascii(handle, desc.iManufacturer,
                                                 string, sizeof(string));
    if (ret > 0) {
      ROS_INFO_STREAM("Manufacturer: " << string);
    }
  }

  if (desc.iProduct) {
    unsigned char string[256];
    int ret = libusb_get_string_descriptor_ascii(handle, desc.iProduct, string,
                                                 sizeof(string));
    if (ret > 0) {
      ROS_INFO_STREAM("Product: " << string);
    }
  }

  if (desc.iSerialNumber) {
    unsigned char string[256];
    int ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                                                 string, sizeof(string));
    if (ret > 0) {
      ROS_INFO_STREAM("SerialNumber: " << string);
    }
  }
}

}  // namespace piksi_multi_cpp
