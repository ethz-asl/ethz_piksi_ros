#include <ros/console.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay_heartbeat.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay_imu_raw.h"

#include <libsbp/ext_events.h>
#include <libsbp/imu.h>
#include <libsbp/system.h>

namespace piksi_multi_cpp {

SBPCallbackHandler::SBPCallbackHandlerPtr
SBPCallbackHandlerFactory::createRelayCallbackBySBPMsgType(
    const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
    const std::shared_ptr<sbp_state_t>& state) {
  switch (sbp_msg_type) {
    case SBP_MSG_IMU_RAW:
      return SBPCallbackHandler::SBPCallbackHandlerPtr(
          new SBPCallbackHandlerRelayImuRaw(nh, SBP_MSG_IMU_RAW, state));
    case SBP_MSG_HEARTBEAT:
      return SBPCallbackHandler::SBPCallbackHandlerPtr(
          new SBPCallbackHandlerRelayHeartbeat(nh, SBP_MSG_HEARTBEAT, state));
    default:
      ROS_WARN("Message type %u not implemented.", sbp_msg_type);
      return nullptr;
  }
}

// Factory method to create all implemented SBP message relays.
std::vector<SBPCallbackHandler::SBPCallbackHandlerPtr>
SBPCallbackHandlerFactory::createAllSBPMessageRelays(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  std::vector<SBPCallbackHandler::SBPCallbackHandlerPtr> cbs;
  // Ext Event
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_EXT_EVENT, state));
  // Imu
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_IMU_RAW, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_IMU_AUX, state));
  // TODO(rikba): Implement all other callbacks.
  // System
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_STARTUP, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_DGNSS_STATUS, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_HEARTBEAT, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_INS_STATUS, state));

  // Remove all invalid (nullptr) callbacks.
  cbs.erase(
      std::remove_if(cbs.begin(), cbs.end(),
                     [](const SBPCallbackHandler::SBPCallbackHandlerPtr& cb) {
                       return cb.get() == nullptr;
                     }));

  return cbs;
}

}  // namespace piksi_multi_cpp
