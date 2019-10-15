#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_TRACKING_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_TRACKING_H_

#include <libsbp/tracking.h>
#include <piksi_multi_msgs/MeasurementStates.h>
#include <piksi_multi_msgs/TrackingStates.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayTrackingState
    : public SBPCallbackHandlerRelay<msg_tracking_state_t,
                                     piksi_multi_msgs::TrackingStates> {
 public:
  inline SBPCallbackHandlerRelayTrackingState(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_TRACKING_STATE, state,
                                "tracking_state") {}
};

class SBPCallbackHandlerRelayMeasurementState
    : public SBPCallbackHandlerRelay<msg_measurement_state_t,
                                     piksi_multi_msgs::MeasurementStates> {
 public:
  inline SBPCallbackHandlerRelayMeasurementState(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_MEASUREMENT_STATE, state,
                                "measurement_state") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_TRACKING_H_
