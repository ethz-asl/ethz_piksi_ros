#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_PIKSI_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_PIKSI_H_

#include <libsbp/piksi.h>
#include <piksi_multi_msgs/DeviceMonitor.h>
#include <piksi_multi_msgs/UartState.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayUartState
    : public SBPCallbackHandlerRelay<msg_uart_state_t,
                                     piksi_multi_msgs::UartState> {
 public:
  inline SBPCallbackHandlerRelayUartState(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_UART_STATE, state, "uart_state") {}
};

class SBPCallbackHandlerRelayDeviceMonitor
    : public SBPCallbackHandlerRelay<msg_device_monitor_t,
                                     piksi_multi_msgs::DeviceMonitor> {
 public:
  inline SBPCallbackHandlerRelayDeviceMonitor(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_DEVICE_MONITOR, state,
                                "device_monitor") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_PIKSI_H_
