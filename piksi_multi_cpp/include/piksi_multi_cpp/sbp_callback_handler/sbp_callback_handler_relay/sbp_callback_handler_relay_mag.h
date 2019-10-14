#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_MAG_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_MAG_H_

#include <libsbp/mag.h>
#include <piksi_multi_msgs/MagRaw.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayMagRaw
    : public SBPCallbackHandlerRelay<msg_mag_raw_t, piksi_multi_msgs::MagRaw> {
 public:
  inline SBPCallbackHandlerRelayMagRaw(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_MAG_RAW, state, "mag_raw") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_MAG_H_
