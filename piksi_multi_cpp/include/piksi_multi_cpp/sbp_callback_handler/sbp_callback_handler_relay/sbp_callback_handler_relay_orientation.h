#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ORIENTATION_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ORIENTATION_H_

#include <libsbp/orientation.h>
#include <piksi_multi_msgs/BaselineHeading.h>
#include <piksi_multi_msgs/OrientationQuat.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayBaselineHeading
    : public SBPCallbackHandlerRelay<msg_baseline_heading_t,
                                     piksi_multi_msgs::BaselineHeading> {
 public:
  inline SBPCallbackHandlerRelayBaselineHeading(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_BASELINE_HEADING, state,
                                "baseline_heading") {}
};

class SBPCallbackHandlerRelayOrientQuat
    : public SBPCallbackHandlerRelay<msg_orient_quat_t,
                                     piksi_multi_msgs::OrientationQuat> {
 public:
  inline SBPCallbackHandlerRelayOrientQuat(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_ORIENT_QUAT, state, "orient_quat") {
  }
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ORIENTATION_H_
