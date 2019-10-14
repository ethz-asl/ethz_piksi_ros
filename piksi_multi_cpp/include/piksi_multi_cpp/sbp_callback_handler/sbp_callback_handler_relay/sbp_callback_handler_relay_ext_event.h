#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_EXT_EVENT_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_EXT_EVENT_H_

#include <libsbp/ext_events.h>
#include <piksi_multi_msgs/ExtEvent.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayExtEvent
    : public SBPCallbackHandlerRelay<msg_ext_event_t,
                                     piksi_multi_msgs::ExtEvent> {
 public:
  inline SBPCallbackHandlerRelayExtEvent(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_EXT_EVENT, state, "ext_event") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_EXT_EVENT_H_
