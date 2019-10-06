#include "piksi_multi_cpp/piksi_multi.h"

#include <libsbp/sbp.h>
#include <libusb-1.0/libusb.h>

namespace piksi_multi_cpp {

PiksiMulti::PiksiMulti(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  getROSParameters();
  advertiseTopics();
}

void PiksiMulti::getROSParameters() {}

void PiksiMulti::advertiseTopics() {}

bool PiksiMulti::open() {
  bool has_usb_devs = true;
  while (has_usb_devs) {
    DeviceUSB new_dev;
    has_usb_devs = new_dev.open();
    if (has_usb_devs) devices_usb_.push_back(new_dev);
  }
  return !devices_usb_.empty();
}

bool PiksiMulti::close() {
  bool success = true;
  for (auto dev : devices_usb_) {
    success = dev.close() && success;
  }

  return success;
}

bool PiksiMulti::read() { return true; }

}  // namespace piksi_multi_cpp
