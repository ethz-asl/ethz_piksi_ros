#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay_imu_raw.h"

namespace piksi_multi_cpp {
// Define ROS and SBP message types.
typedef piksi_rtk_msgs::ImuRawMulti ROSMSgType;
typedef msg_imu_raw_t SBPMSgType;
const std::string kTopic = "imu_raw";

// Here we define the topic name.
SBPCallbackHandlerRelayImuRaw::SBPCallbackHandlerRelayImuRaw(
    const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
    const std::shared_ptr<sbp_state_t>& state)
    : SBPCallbackHandlerRelay(nh, sbp_msg_type, state, kTopic) {}

// The SBP message is translated into a ROS message here.
ROSMSgType SBPCallbackHandlerRelayImuRaw::convertSBPMsgToROSMsg(
    const SBPMSgType& sbp_msg) {
  ROSMSgType ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.tow_f = sbp_msg.tow_f;
  ros_msg.acc_x = sbp_msg.acc_x;
  ros_msg.acc_y = sbp_msg.acc_y;
  ros_msg.acc_z = sbp_msg.acc_z;
  ros_msg.gyr_x = sbp_msg.gyr_x;
  ros_msg.gyr_y = sbp_msg.gyr_y;
  ros_msg.gyr_z = sbp_msg.gyr_z;

  return ros_msg;
}

}  // namespace piksi_multi_cpp
