#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SYSTEM_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SYSTEM_H_

#include <libsbp/system.h>
#include <piksi_multi_msgs/DgnssStatus.h>
#include <piksi_multi_msgs/Heartbeat.h>
#include <piksi_multi_msgs/InsStatus.h>
#include <piksi_multi_msgs/Startup.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayStartup
    : public SBPCallbackHandlerRelay<msg_startup_t, piksi_multi_msgs::Startup> {
 public:
  inline SBPCallbackHandlerRelayStartup(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_STARTUP, state, "startup") {}
};

class SBPCallbackHandlerRelayDgnssStatus
    : public SBPCallbackHandlerRelay<msg_dgnss_status_t,
                                     piksi_multi_msgs::DgnssStatus> {
 public:
  inline SBPCallbackHandlerRelayDgnssStatus(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_DGNSS_STATUS, state,
                                "dgnss_status") {}
};

class SBPCallbackHandlerRelayHeartbeat
    : public SBPCallbackHandlerRelay<msg_heartbeat_t,
                                     piksi_multi_msgs::Heartbeat> {
 public:
  inline SBPCallbackHandlerRelayHeartbeat(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_HEARTBEAT, state, "heartbeat") {}
};

class SBPCallbackHandlerRelayInsStatus
    : public SBPCallbackHandlerRelay<msg_ins_status_t,
                                     piksi_multi_msgs::InsStatus> {
 public:
  inline SBPCallbackHandlerRelayInsStatus(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_INS_STATUS, state, "ins_status") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SYSTEM_H_
