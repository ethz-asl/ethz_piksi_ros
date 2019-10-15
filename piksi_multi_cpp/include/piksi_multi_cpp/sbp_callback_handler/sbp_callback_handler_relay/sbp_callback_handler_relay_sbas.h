#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBAS_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBAS_H_

#include <libsbp/sbas.h>
#include <piksi_multi_msgs/SbasRaw.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelaySbasRaw
    : public SBPCallbackHandlerRelay<msg_sbas_raw_t,
                                     piksi_multi_msgs::SbasRaw> {
 public:
  inline SBPCallbackHandlerRelaySbasRaw(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_SBAS_RAW, state, "sbas_raw") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBAS_H_
