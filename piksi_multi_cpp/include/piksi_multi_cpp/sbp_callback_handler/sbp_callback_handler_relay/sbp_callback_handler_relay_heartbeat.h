#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_HEARTBEAT_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_HEARTBEAT_H_

#include <libsbp/system.h>
#include <piksi_multi_msgs/Heartbeat.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayHeartbeat
    : public SBPCallbackHandlerRelay<msg_heartbeat_t,
                                     piksi_multi_msgs::Heartbeat> {
 public:
  inline SBPCallbackHandlerRelayHeartbeat(
      const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
      const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, sbp_msg_type, state, "heartbeat") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_HEARTBEAT_H_
