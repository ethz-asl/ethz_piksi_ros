#include <libsbp/system.h>
#include <ros/console.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_heartbeat.h"

namespace piksi_multi_cpp {

SBPCallbackHandler::SBPCallbackHandlerPtr
SBPCallbackHandlerFactory::createRelayCallbackBySBPMsgType(
    const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
    const std::shared_ptr<sbp_state_t>& state) {
  switch (sbp_msg_type) {
    case SBP_MSG_HEARTBEAT:
      return SBPCallbackHandler::SBPCallbackHandlerPtr(
          new SBPCallbackHeartbeat(nh, SBP_MSG_HEARTBEAT, state));
    default:
      ROS_ERROR("Message type %u not implemented.", sbp_msg_type);
      return nullptr;
  }
}

}  // namespace piksi_multi_cpp
