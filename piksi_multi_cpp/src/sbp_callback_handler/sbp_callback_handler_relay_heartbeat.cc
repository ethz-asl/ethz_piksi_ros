#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay_heartbeat.h"

#include <libsbp/system.h>

namespace piksi_multi_cpp {

// Here we define the topic name.
SBPCallbackHandlerRelayHeartbeat::SBPCallbackHandlerRelayHeartbeat(
    const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
    const std::shared_ptr<sbp_state_t>& state)
    : SBPCallbackHandlerRelay(nh, sbp_msg_type, state, "heartbeat") {}

// The SBP message is translated into a ROS message here.
piksi_rtk_msgs::Heartbeat
SBPCallbackHandlerRelayHeartbeat::convertSBPMsgToROSMsg(
    const msg_heartbeat_t& sbp_msg) {
  piksi_rtk_msgs::Heartbeat ros_msg;
  ros_msg.header.stamp = ros::Time::now();

  // How to shift and mask bits: https://stackoverflow.com/a/27592777
  ros_msg.system_error = (sbp_msg.flags >> 0) & 0x1;
  ros_msg.io_error = (sbp_msg.flags >> 1) & 0x1;
  ros_msg.swift_nap_error = (sbp_msg.flags >> 2) & 0x1;
  ros_msg.sbp_minor_version = (sbp_msg.flags >> 8) & 0xFF;
  ros_msg.sbp_major_version = (sbp_msg.flags >> 16) & 0xFF;
  ros_msg.external_antenna_present = (sbp_msg.flags >> 31) & 0x1;

  return ros_msg;
}

}  // namespace piksi_multi_cpp
