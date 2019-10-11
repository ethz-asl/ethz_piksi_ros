#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_IMU_RAW_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_IMU_RAW_H_

#include <libsbp/imu.h>
#include <piksi_rtk_msgs/ImuRawMulti.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayImuRaw
    : public SBPCallbackHandlerRelay<piksi_rtk_msgs::ImuRawMulti,
                                     msg_imu_raw_t> {
 public:
  SBPCallbackHandlerRelayImuRaw(const ros::NodeHandle& nh,
                                const uint16_t sbp_msg_type,
                                const std::shared_ptr<sbp_state_t>& state);

 private:
  // Specialize the message conversion.
  piksi_rtk_msgs::ImuRawMulti convertSBPMsgToROSMsg(const msg_imu_raw_t& sbp_msg) override;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_IMU_RAW_H_
