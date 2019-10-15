#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_LINUX_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_LINUX_H_

#include <libsbp/linux.h>
#include <piksi_multi_msgs/CpuState.h>
#include <piksi_multi_msgs/MemState.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayCpuState
    : public SBPCallbackHandlerRelay<msg_linux_cpu_state_t,
                                     piksi_multi_msgs::CpuState> {
 public:
  inline SBPCallbackHandlerRelayCpuState(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_CPU_STATE, state,
                                "cpu_state") {}
};

class SBPCallbackHandlerRelayMemState
    : public SBPCallbackHandlerRelay<msg_linux_mem_state_t,
                                     piksi_multi_msgs::MemState> {
 public:
  inline SBPCallbackHandlerRelayMemState(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_MEM_STATE, state,
                                "mem_state") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_LINUX_H_
