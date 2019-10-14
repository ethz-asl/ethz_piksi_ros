#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_IMU_RAW_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_IMU_RAW_H_

#include <libsbp/imu.h>
#include <piksi_multi_msgs/ImuRaw.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

// Specialize callback for piksi_multi_msgs::ImuRaw ROS msg type and
// msg_imu_raw_t SBP msg type.
class SBPCallbackHandlerRelayImuRaw
    : public SBPCallbackHandlerRelay<msg_imu_raw_t, piksi_multi_msgs::ImuRaw> {
 public:
  // Here we define the topic name.
  inline SBPCallbackHandlerRelayImuRaw(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_IMU_RAW, state, "imu_raw") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_IMU_RAW_H_
