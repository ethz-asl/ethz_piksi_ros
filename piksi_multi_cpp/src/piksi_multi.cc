#include "piksi_multi_cpp/piksi_multi.h"

#include <libsbp/sbp.h>
#include <libusb-1.0/libusb.h>

namespace piksi_multi_cpp {

PiksiMulti::PiksiMulti(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), interface_(Interface::kInvalid) {
  getROSParameters();
  advertiseTopics();
}

void PiksiMulti::getROSParameters() {
  nh_private_.getParam("interface", params_.interface);
  if (kStrToInterface.find(params_.interface) != kStrToInterface.end()) {
    interface_ = kStrToInterface.find(params_.interface)->second;
  }
  nh_private_.getParam("serial_port", params_.serial_port);
}

void PiksiMulti::advertiseTopics() {}

bool PiksiMulti::connect() {
  // Connect to interface.
  switch (interface_) {
    case Interface::kUSB:
      return connectUSB();
    case Interface::kSerial:
      return connectSerial();
    case Interface::kTCP:
      return connectTCP();
    default:
      ROS_ERROR("No valid interface ('usb', 'serial', 'tcp') specified.");
      return false;
  }
}
bool PiksiMulti::disconnect() {
  // Connect to interface.
  switch (interface_) {
    case Interface::kUSB:
      return disconnectUSB();
    case Interface::kSerial:
      return disconnectSerial();
    case Interface::kTCP:
      return disconnectTCP();
    default:
      ROS_ERROR("No valid interface ('usb', 'serial', 'tcp') specified.");
      return false;
  }
}

bool PiksiMulti::read() {
  // Read interface.
  switch (interface_) {
    case Interface::kUSB:
      return readUSB();
    case Interface::kSerial:
      return readSerial();
    case Interface::kTCP:
      return readTCP();
    default:
      ROS_ERROR("No valid interface ('usb', 'serial', 'tcp') specified.");
      return false;
  }
}

bool PiksiMulti::connectUSB() {
  // Initialize libusb.
  // TODO(rikba): Set context to allow multiple USB users.
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
    if (identifyUSB(device)) {
      piksis.push_back(device);
    }
  }
  if (piksis.empty()) {
    ROS_INFO("No Piksi discovered on USB.");
    return false;
  }
  ROS_INFO_STREAM("Discovered " << piksis.size() << " Piksi(s) on USB.");

  // Create libusb_devices structure.
  libusb_devices_ = LibUSBDevices();

  // Open devices.
  for (auto piksi : piksis) {
    libusb_device_handle* handle;
    int open = libusb_open(piksi, &handle);
    if (open < 0) {
      ROS_WARN_STREAM("Failed to open:" << open);
      continue;
    }
    libusb_devices_.handles.push_back(handle);
  }

  libusb_free_device_list(list, 1);

  return !libusb_devices_.handles.empty();
}

bool PiksiMulti::connectSerial() {
  ROS_ERROR("Serial interface not implemented.");
  return false;
}

bool PiksiMulti::connectTCP() {
  ROS_ERROR("TCP interface not implemented.");
  return false;
}

bool PiksiMulti::identifyUSB(libusb_device* dev) {
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
    return true;
  } else {
    return false;
  }
}

bool PiksiMulti::readUSB() {
  ROS_ERROR("USB interface not implemented.");
  return false;
}

bool PiksiMulti::readSerial() {
  ROS_ERROR("Serial interface not implemented.");
  return false;
}
bool PiksiMulti::readTCP() {
  ROS_ERROR("TCP interface not implemented.");
  return false;
}

bool PiksiMulti::disconnectUSB() {
  // TODO(rikba): Set context to allow multiple USB users.
  libusb_exit(NULL);

  return true;
}

bool PiksiMulti::disconnectSerial() {
  ROS_ERROR("Serial interface not implemented.");
  return false;
}

bool PiksiMulti::disconnectTCP() {
  ROS_ERROR("TCP interface not implemented.");
  return false;
}

}  // namespace piksi_multi_cpp
