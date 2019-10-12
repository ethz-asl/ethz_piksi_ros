#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_HEARTBEAT_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_HEARTBEAT_H_

#include <libsbp/system.h>
#include <piksi_rtk_msgs/Heartbeat.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayHeartbeat
    : public SBPCallbackHandlerRelay<piksi_rtk_msgs::Heartbeat,
                                     msg_heartbeat_t> {
 public:
  inline SBPCallbackHandlerRelayHeartbeat(
      const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
      const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, sbp_msg_type, state, "heartbeat") {}

 private:
  // Specialize the message conversion.
  inline piksi_rtk_msgs::Heartbeat convertSBPMsgToROSMsg(
      const msg_heartbeat_t& sbp_msg) override {
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
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_HEARTBEAT_H_
