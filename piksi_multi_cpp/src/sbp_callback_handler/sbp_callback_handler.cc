#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

#include <ros/console.h>

namespace piksi_multi_cpp {

SBPCallbackHandler::SBPCallbackHandler(
    const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
    const std::shared_ptr<sbp_state_t>& state)
    : nh_(nh), state_(state) {
  sbp_register_callback(state.get(), sbp_msg_type,
                        &piksi_multi_cpp::SBPCallbackHandler::callback_redirect,
                        this, &sbp_msg_callback_node_);
}

void SBPCallbackHandler::callback_redirect(uint16_t sender_id, uint8_t len,
                                           uint8_t msg[], void* context) {
  if (!context) {
    ROS_ERROR("Context not set.");
    return;
  }
  // Cast context to instance.
  SBPCallbackHandler* instance = static_cast<SBPCallbackHandler*>(context);
  // Execute instance's read function.
  return instance->callback(sender_id, len, msg);
}

}  // namespace piksi_multi_cpp
