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

bool PiksiMulti::open() { return devices_usb_.open(); }
bool PiksiMulti::close() { return devices_usb_.close(); }
bool PiksiMulti::read() { return devices_usb_.read(); }

}  // namespace piksi_multi_cpp
