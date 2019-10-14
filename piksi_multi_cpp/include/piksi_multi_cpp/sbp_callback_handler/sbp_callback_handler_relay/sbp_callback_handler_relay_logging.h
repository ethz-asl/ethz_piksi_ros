#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_LOGGING_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_LOGGING_H_

#include <libsbp/logging.h>
#include <piksi_multi_msgs/Fwd.h>
#include <piksi_multi_msgs/Log.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayLog
    : public SBPCallbackHandlerRelay<msg_log_t, piksi_multi_msgs::Log> {
 public:
  inline SBPCallbackHandlerRelayLog(const ros::NodeHandle& nh,
                                    const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_LOG, state, "log") {}
};

class SBPCallbackHandlerRelayFwd
    : public SBPCallbackHandlerRelay<msg_fwd_t, piksi_multi_msgs::Fwd> {
 public:
  inline SBPCallbackHandlerRelayFwd(const ros::NodeHandle& nh,
                                    const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_FWD, state, "fwd") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_LOGGING_H_
