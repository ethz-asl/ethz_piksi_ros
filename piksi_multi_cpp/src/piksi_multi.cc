#include "piksi_multi_cpp/piksi_multi.h"

#include <libsbp/sbp.h>
#include <libsbp/system.h>

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
  for (auto dev : devices_) {
    if (!dev.get()) continue;
    dev->open();
  }

  return !devices_.empty();
}

void PiksiMulti::close() {
  // Close all devices.
  for (auto dev : devices_) {
    if (!dev.get()) continue;
    dev->close();
  }
}

void PiksiMulti::read() {
  // Read all devices sequentially.
  // TODO(rikba): Multithreading.
  for (auto dev : devices_) {
    if (!dev.get()) continue;
    int result =
        sbp_process(&states_[dev], &piksi_multi_cpp::Device::read_redirect);
    if (result < 0) {
      ROS_WARN_STREAM("Error reading data: " << result);
    }
  }
}

}  // namespace piksi_multi_cpp
