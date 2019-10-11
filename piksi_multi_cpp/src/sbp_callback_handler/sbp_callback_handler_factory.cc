#include <ros/console.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay_heartbeat.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay_imu_raw.h"

#include <libsbp/ext_events.h>
#include <libsbp/imu.h>
#include <libsbp/system.h>

namespace piksi_multi_cpp {

SBPCallbackHandler::SBPCallbackHandlerPtr
SBPCallbackHandlerFactory::createSBPRelayCallbackBySBPMsgType(
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
  auto cb = createSBPRelayCallbackBySBPMsgType(nh, SBP_MSG_EXT_EVENT, state);
  if (cb.get()) cbs.push_back(cb);
  // Imu
  cb = createSBPRelayCallbackBySBPMsgType(nh, SBP_MSG_IMU_RAW, state);
  if (cb.get()) cbs.push_back(cb);
  cb = createSBPRelayCallbackBySBPMsgType(nh, SBP_MSG_IMU_AUX, state);
  if (cb.get()) cbs.push_back(cb);
  // TODO(rikba): Implement all other callbacks.
  // System
  cb = createSBPRelayCallbackBySBPMsgType(nh, SBP_MSG_STARTUP, state);
  if (cb.get()) cbs.push_back(cb);
  cb = createSBPRelayCallbackBySBPMsgType(nh, SBP_MSG_DGNSS_STATUS, state);
  if (cb.get()) cbs.push_back(cb);
  cb = createSBPRelayCallbackBySBPMsgType(nh, SBP_MSG_HEARTBEAT, state);
  if (cb.get()) cbs.push_back(cb);
  cb = createSBPRelayCallbackBySBPMsgType(nh, SBP_MSG_INS_STATUS, state);
  if (cb.get()) cbs.push_back(cb);

  return cbs;
}

}  // namespace piksi_multi_cpp
