#include "piksi_multi_cpp/piksi_multi.h"

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libusb-1.0/libusb.h>
#include <memory>

namespace piksi_multi_cpp {

PiksiMulti::PiksiMulti(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  getROSParameters();
  advertiseTopics();

  // Create all devices.
  devices_ = Device::createAllDevices();
  // Initialize SBP.
  for (auto dev : devices_) {
    states_[dev] = sbp_state_t();
    sbp_state_init(&states_[dev]);

    // Standard callbacks for all devices.
    heartbeat_callback_nodes_[dev] = sbp_msg_callbacks_node_t();
    sbp_register_callback(&states_[dev], SBP_MSG_HEARTBEAT,
                          &piksi_multi_cpp::PiksiMulti::callbackHeartbeat, this,
                          &heartbeat_callback_nodes_[dev]);
  }
}

void PiksiMulti::getROSParameters() {}

void PiksiMulti::advertiseTopics() {}

void PiksiMulti::callbackHeartbeat(uint16_t sender_id, uint8_t len,
                                   uint8_t msg[], void* context) {
  (void)sender_id, (void)len, (void)msg, (void)context;
  fprintf(stdout, "%s\n", __FUNCTION__);
  ROS_INFO("TEST");
}

bool PiksiMulti::open() {
  // // Discover and open all attached USB devices.
  // bool has_usb_devs = true;
  // while (has_usb_devs) {
  //   auto new_dev = std::make_shared<DeviceUSB>();
  //   has_usb_devs = new_dev->open();
  //   if (has_usb_devs) {
  //     devices_.push_back(std::static_pointer_cast<Device>(new_dev));
  //   }
  // }

  return !devices_.empty();
}

bool PiksiMulti::close() {
  bool success = true;
  // // Close all devices.
  // for (auto dev : devices_) {
  //   if (!dev) continue;
  //   success = dev->close() && success;
  // }

  return success;
}

void PiksiMulti::read() {
  // for (auto dev : devices_) {
  //   if (!dev) continue;
  //   current_device_ = dev.get();
  //   sbp_state_set_io_context(&state_, current_device_);
  //
  //   // TODO(rikba): It would be nice to be able to call base class read
  //   instead
  //   // of trying to cast all devices. sbp_process(&state_,
  //   // &piksi_multi_cpp::Device::read);
  //
  //   auto usb_dev = std::dynamic_pointer_cast<DeviceUSB>(dev);
  //   if (usb_dev) {
  //     int process = sbp_process(&state_, &piksi_multi_cpp::DeviceUSB::read);
  //     ROS_INFO_STREAM("process: " << process);
  //   }
  // }
}

}  // namespace piksi_multi_cpp
