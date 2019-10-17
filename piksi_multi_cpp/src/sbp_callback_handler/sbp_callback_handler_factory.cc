#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h"
#include <libsbp_ros_msgs/sbp_callback_handler_relay_sbp.h>

namespace piksi_multi_cpp {

// Factory method to create all implemented SBP message relays.
std::vector<SBPCallbackHandler::Ptr>
SBPCallbackHandlerFactory::createAllSBPMessageRelays(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {

  return createAllSbpMsgRelays(nh, state);
}

}  // namespace piksi_multi_cpp
