#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

#include <ros/console.h>

namespace piksi_multi_cpp {

SBPCallbackHandler::SBPCallbackHandler(
    const uint16_t sbp_msg_type, const std::shared_ptr<sbp_state_t>& state)
    : state_(state) {
  int result = sbp_register_callback(
      state.get(), sbp_msg_type,
      &piksi_multi_cpp::SBPCallbackHandler::callback_redirect, this,
      &sbp_msg_callback_node_);
  ROS_ERROR_COND(result != SBP_OK,
                 "Could not register callback for message type %.4X err %d",
                 sbp_msg_type, result);
}

SBPCallbackHandler::~SBPCallbackHandler() {
  int result = sbp_remove_callback(state_.get(), &sbp_msg_callback_node_);
  ROS_ERROR_COND(result != SBP_OK, "Failed to remove callback %d", result);
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
