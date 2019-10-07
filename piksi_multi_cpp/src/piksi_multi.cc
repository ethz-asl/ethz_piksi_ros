#include "piksi_multi_cpp/piksi_multi.h"

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <piksi_rtk_msgs/Heartbeat.h>

namespace piksi_multi_cpp {

const bool kQueueSize = 100;
const bool kLatchTopic = true;

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

    // Standard callbacks for all device types.
    // TODO(rikba): Replace device address with actual device type and number,
    // e.g., reference_receiver_1 or attitude_receiver_0 or basestation_0.
    const std::string kDevPrefix = "dev_" + dev->getID();
    // Heartbeat.
    // Specify all publishers required in heartbeat callback. Here we have one
    // publisher relaying the heartbeat message.
    ros_publishers_[dev][SBP_MSG_HEARTBEAT] = std::vector<ros::Publisher>(
        {nh_private_.advertise<piksi_rtk_msgs::Heartbeat>(
            kDevPrefix + "/heartbeat", kQueueSize, kLatchTopic)});
    // Create SBP msg callback node.
    heartbeat_callback_nodes_[dev] = sbp_msg_callbacks_node_t();
    // Create SBP msg callback. Input arguments are the state of the device, the
    // message type, the callback function, a pointer to all publishers for this
    // function and the node we just created.
    sbp_register_callback(&states_[dev], SBP_MSG_HEARTBEAT,
                          &piksi_multi_cpp::PiksiMulti::callbackHeartbeat,
                          &ros_publishers_[dev][SBP_MSG_HEARTBEAT],
                          &heartbeat_callback_nodes_[dev]);
  }
}  // namespace piksi_multi_cpp

void PiksiMulti::getROSParameters() {}

void PiksiMulti::advertiseTopics() {}

void PiksiMulti::callbackHeartbeat(uint16_t sender_id, uint8_t len,
                                   uint8_t msg[], void* context) {
  auto sbp_msg_heartbeat = (msg_heartbeat_t*)msg;
  if (!sbp_msg_heartbeat) return;

  piksi_rtk_msgs::Heartbeat ros_msg_heartbeat;
  ros_msg_heartbeat.header.stamp = ros::Time::now();

  // How to shift and mask bits: https://stackoverflow.com/a/27592777
  ros_msg_heartbeat.system_error = (sbp_msg_heartbeat->flags >> 0) & 0x1;
  ros_msg_heartbeat.io_error = (sbp_msg_heartbeat->flags >> 1) & 0x1;
  ros_msg_heartbeat.swift_nap_error = (sbp_msg_heartbeat->flags >> 2) & 0x1;
  ros_msg_heartbeat.sbp_minor_version = (sbp_msg_heartbeat->flags >> 8) & 0xFF;
  ros_msg_heartbeat.sbp_major_version = (sbp_msg_heartbeat->flags >> 16) & 0xFF;
  ros_msg_heartbeat.external_antenna_present =
      (sbp_msg_heartbeat->flags >> 31) & 0x1;

  auto pub_vec = static_cast<std::vector<ros::Publisher>*>(context);
  if (!pub_vec) return;
  pub_vec->front().publish(ros_msg_heartbeat);
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
    sbp_state_set_io_context(&states_[dev], dev.get());
    int result =
        sbp_process(&states_[dev], &piksi_multi_cpp::Device::read_redirect);
    if (result < 0) {
      ROS_WARN_STREAM("Error reading data: " << result);
    }
  }
}

}  // namespace piksi_multi_cpp
