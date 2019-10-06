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
    close();
    return false;
  }

  // Discover all devices.
  libusb_device** list;
  ssize_t cnt = libusb_get_device_list(nullptr, &list);
  if (cnt < 0) {
    ROS_ERROR_STREAM("Invalid device list: " << cnt);
    close();
    return false;
  }

  bool new_dev = false;
  for (ssize_t i = 0; i < cnt; ++i) {
    // Identify Piksi.
    libusb_device* device = list[i];
    if (!identifyPiksi(device)) continue;

    // Open port.
    int open = libusb_open(device, &handle_);
    if (open < 0) {
      ROS_INFO(
          "Failed to open Piksi Multi on bus number / address %03d.%03d: %d",
          libusb_get_bus_number(device), libusb_get_device_address(device),
          open);
      continue;
    }

    // Claim device.
    int detach_kernel = libusb_set_auto_detach_kernel_driver(handle_, 1);
    if (detach_kernel < 0) {
      ROS_WARN_STREAM("Cannot detach kernel driver: " << detach_kernel);
    }
    int claim = libusb_claim_interface(handle_, kInterfaceNumber);
    if (claim < 0) {
      ROS_INFO_STREAM("Failed to claim device: " << claim);
      ROS_INFO_COND(claim == LIBUSB_ERROR_BUSY, "Device already claimed.");
      continue;
    }

    // Save device handle.
    ROS_INFO_STREAM("Opened new device:");
    new_dev = true;
    printDeviceInfo(handle_);
    break;
  }

  libusb_free_device_list(list, 1);

  if (new_dev) {
    return true;
  } else {
    close();
    return false;
  }
}

bool DeviceUSB::read() {
  ROS_ERROR("Read USB not implemented.");
  return false;
}

bool DeviceUSB::close() {
  bool success = true;
  if (handle_) {
    int release = libusb_release_interface(handle_, kInterfaceNumber);
    if (release < 0) {
      ROS_WARN_STREAM("Device was not released: " << release);
      success = false;
    }
    libusb_close(handle_);
  }
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
