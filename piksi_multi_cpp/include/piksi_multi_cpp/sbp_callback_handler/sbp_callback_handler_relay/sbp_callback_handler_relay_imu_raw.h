#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_IMU_RAW_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_IMU_RAW_H_

#include <libsbp/imu.h>
#include <piksi_rtk_msgs/ImuRawMulti.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

// Specialize callback for piksi_rtk_msgs::ImuRawMulti ROS msg type and
// msg_imu_raw_t SBP msg type.
class SBPCallbackHandlerRelayImuRaw
    : public SBPCallbackHandlerRelay<piksi_rtk_msgs::ImuRawMulti,
                                     msg_imu_raw_t> {
 public:
  // Here we define the topic name.
  inline SBPCallbackHandlerRelayImuRaw(
      const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
      const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, sbp_msg_type, state, "imu_raw") {}

 private:
  // Specialize the message conversion.
  inline piksi_rtk_msgs::ImuRawMulti convertSBPMsgToROSMsg(
      const msg_imu_raw_t& sbp_msg) override {
    piksi_rtk_msgs::ImuRawMulti ros_msg;
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
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_IMU_RAW_H_
