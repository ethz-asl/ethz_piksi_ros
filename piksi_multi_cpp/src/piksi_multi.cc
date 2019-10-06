#include "piksi_multi_cpp/piksi_multi.h"

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libusb-1.0/libusb.h>

namespace piksi_multi_cpp {

PiksiMulti::PiksiMulti(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  getROSParameters();
  advertiseTopics();

  // Setup SBP.
  sbp_state_init(&state_);
  sbp_register_callback(&state_, SBP_MSG_HEARTBEAT,
                        &piksi_multi_cpp::PiksiMulti::callbackHeartbeat, NULL,
                        &heartbeat_callback_node_);
}

void PiksiMulti::getROSParameters() {}

void PiksiMulti::advertiseTopics() {}

void PiksiMulti::callbackHeartbeat(uint16_t sender_id, uint8_t len,
                                   uint8_t msg[], void* context) {
  (void)sender_id, (void)len, (void)msg, (void)context;
  fprintf(stdout, "%s\n", __FUNCTION__);
}

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

void PiksiMulti::read() {
  for (auto dev : devices_usb_) {
    sbp_process(&state_, &dev.read);
  }
}

}  // namespace piksi_multi_cpp
