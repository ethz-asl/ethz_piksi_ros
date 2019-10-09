#include "piksi_multi_cpp/ros_output.h"

#include <ros/console.h>

// Forward declaration.
#include <libsbp/system.h>
#include "piksi_multi_cpp/ros_output_heartbeat.h"

namespace piksi_multi_cpp {

ROSOutput::ROSOutput(const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
                     const std::shared_ptr<sbp_state_t>& state)
    : nh_(nh), state_(state) {
  sbp_register_callback(state.get(), sbp_msg_type,
                        &piksi_multi_cpp::ROSOutputHeartbeat::callback_redirect,
                        this, &sbp_msg_callback_node_);
}

void ROSOutput::callback_redirect(uint16_t sender_id, uint8_t len,
                                  uint8_t msg[], void* context) {
  if (!context) {
    ROS_ERROR("Context not set.");
    return;
  }
  // Cast context to instance.
  ROSOutput* instance = static_cast<ROSOutput*>(context);
  // Execute instance's read function.
  return instance->callback(sender_id, len, msg);
}

std::shared_ptr<ROSOutput> ROSOutput::create(
    const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
    const std::shared_ptr<sbp_state_t>& state) {
  switch (sbp_msg_type) {
    case SBP_MSG_HEARTBEAT:
      return std::shared_ptr<ROSOutput>(
          new ROSOutputHeartbeat(nh, SBP_MSG_HEARTBEAT, state));
    default:
      ROS_ERROR("Message type %u not implemented.", sbp_msg_type);
      return nullptr;
  }
}

}  // namespace piksi_multi_cpp
