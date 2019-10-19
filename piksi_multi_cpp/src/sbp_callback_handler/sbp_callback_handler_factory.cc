#include <libsbp_ros_msgs/sbp_relays.h>
#include <memory>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h"
#include "piksi_multi_cpp/sbp_callback_handler/utc_time_buffer.h"

#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_ext_event_relay.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_relays.h"

namespace piksi_multi_cpp {

// Factory method to create all implemented SBP message relays.
std::vector<SBPCallbackHandler::Ptr>
SBPCallbackHandlerFactory::createAllSBPMessageRelays(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  return createAllSbpRelays(nh, state);
}

std::vector<SBPCallbackHandler::Ptr>
SBPCallbackHandlerFactory::createAllRosMessageRelays(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  // Enable GPS time stamping.
  std::shared_ptr<UtcTimeBuffer> utc_time_buffer;
  bool use_gps_time = false;
  ros::NodeHandle nh_node("~");
  nh_node.getParam("use_gps_time", use_gps_time);
  if (use_gps_time) {
    utc_time_buffer.reset(new UtcTimeBuffer(nh, state));
  }

  // Create all relays.
  std::vector<SBPCallbackHandler::Ptr> relays;

  relays.push_back(SBPCallbackHandler::Ptr(
      new RosExtEventRelay(nh, state, utc_time_buffer)));
  relays.push_back(
      SBPCallbackHandler::Ptr(new RosPosEcefRelay(nh, state, utc_time_buffer)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosPosEcefCovRelay(nh, state, utc_time_buffer)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosPosLlhCovRelay(nh, state, utc_time_buffer)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosBaselineNedRelay(nh, state, utc_time_buffer)));

  return relays;
}

SBPCallbackHandler::Ptr SBPCallbackHandlerFactory::createUtcTimeBuffer(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  return SBPCallbackHandler::Ptr(new UtcTimeBuffer(nh, state));
}

}  // namespace piksi_multi_cpp
