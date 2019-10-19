#include <libsbp_ros_msgs/sbp_relays.h>
#include <memory>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/utc_time_buffer.h"

#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_navigation_relays.h"

namespace piksi_multi_cpp {

// Factory method to create all implemented SBP message relays.
std::vector<SBPCallbackHandler::Ptr>
SBPCallbackHandlerFactory::createAllSBPMessageRelays(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  return createAllSbpRelays(nh, state);
}

std::vector<SBPCallbackHandler::Ptr>
SBPCallbackHandlerFactory::createAllRosMessageRelays(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state,
    const SBPCallbackHandler::Ptr& utc_time_buffer) {
  // Cast utc time buffer.
  auto utc_time_buffer_cast =
      std::dynamic_pointer_cast<UtcTimeBuffer>(utc_time_buffer);
  if (utc_time_buffer_cast.get() != utc_time_buffer.get()) {
    ROS_ERROR("Failed to cast UtcTimeBuffer.");
    utc_time_buffer_cast.reset();
  }

  std::vector<SBPCallbackHandler::Ptr> relays;

  relays.push_back(SBPCallbackHandler::Ptr(
      new RosPosEcefRelay(nh, state, utc_time_buffer_cast)));

  return relays;
}

SBPCallbackHandler::Ptr SBPCallbackHandlerFactory::createUtcTimeBuffer(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  return SBPCallbackHandler::Ptr(new UtcTimeBuffer(nh, state));
}

}  // namespace piksi_multi_cpp
