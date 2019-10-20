#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_RECEIVER_STATE_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_RECEIVER_STATE_H_

#include <libsbp/navigation.h>
#include <libsbp/tracking.h>
#include <memory.h>
#include <piksi_rtk_msgs/ReceiverState_V2_6_5.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_time_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h"

namespace piksi_multi_cpp {

class RosReceiverState
    : public SBPCallbackHandlerRelay<msg_measurement_state_t,
                                     piksi_rtk_msgs::ReceiverState_V2_6_5> {
 public:
  typedef std::shared_ptr<RosReceiverState> Ptr;

  RosReceiverState(const ros::NodeHandle& nh,
                   const std::shared_ptr<sbp_state_t>& state,
                   const RosTimeHandler::Ptr& ros_time_handler);

  uint16_t getNavSatServiceStatus() const;

 private:
  typedef piksi_rtk_msgs::ReceiverState_V2_6_5 ReceiverState;
  bool convertSbpToRos(const msg_measurement_state_t& sbp_msg,
                       const uint8_t len, ReceiverState* ros_msg) override;

  void callbackToHeartbeat(const msg_heartbeat_t& msg);

  template <class GnssMsgType>
  inline void callbackToGnssSolution(const GnssMsgType& gnss_msg) {
    uint8_t fix_mode = (gnss_msg.flags >> 0) & 0x7;

    switch (fix_mode) {
      case 0:
        receiver_state_.fix_mode = ReceiverState::STR_FIX_MODE_INVALID;
        break;
      case 1:
        receiver_state_.fix_mode = ReceiverState::STR_FIX_MODE_SPP;
        break;
      case 2:
        receiver_state_.fix_mode = ReceiverState::STR_FIX_MODE_DGNSS;
        break;
      case 3:
        receiver_state_.rtk_mode_fix = 0;
        receiver_state_.fix_mode = ReceiverState::STR_FIX_MODE_FLOAT_RTK;
        break;
      case 4:
        receiver_state_.rtk_mode_fix = 1;
        receiver_state_.fix_mode = ReceiverState::STR_FIX_MODE_FIXED_RTK;
        break;
      case 5:
        receiver_state_.fix_mode = ReceiverState::STR_FIX_MODE_DEAD_RECKONING;
        break;
      case 6:
        receiver_state_.fix_mode = ReceiverState::STR_FIX_MODE_SBAS;
        break;
      default:
        receiver_state_.fix_mode = ReceiverState::STR_FIX_MODE_UNKNOWN;
    }
  }

  void resetMeasurementState();

  RosTimeHandler::Ptr ros_time_handler_;
  SBPLambdaCallbackHandler<msg_pos_ecef_t> pos_ecef_handler_;
  SBPLambdaCallbackHandler<msg_pos_ecef_cov_t> pos_ecef_cov_handler_;
  SBPLambdaCallbackHandler<msg_pos_llh_t> pos_llh_handler_;
  SBPLambdaCallbackHandler<msg_pos_llh_cov_t> pos_llh_cov_handler_;
  SBPLambdaCallbackHandler<msg_heartbeat_t> heartbeat_handler_;

  ReceiverState receiver_state_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_RECEIVER_STATE_H_
