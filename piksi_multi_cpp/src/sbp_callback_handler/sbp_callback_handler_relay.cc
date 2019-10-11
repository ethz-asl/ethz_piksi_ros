#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

SBPCallbackHandlerRelay::SBPCallbackHandlerRelay(
    const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
    const std::shared_ptr<sbp_state_t>& state)
    : SBPCallbackHandler(nh, sbp_msg_type, state) {
  // TODO(rikba): advertise topics here using template class.
}

}  // namespace piksi_multi_cpp
