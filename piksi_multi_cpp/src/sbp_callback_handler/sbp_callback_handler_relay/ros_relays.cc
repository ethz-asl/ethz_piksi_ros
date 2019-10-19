#include "sbp_callback_handler/sbp_callback_handler_relay/ros_position_relay.h"

namespace piksi_multi_cpp {

RosPositionRelay::RosPositionRelay(const ros::NodeHandle& nh,
                                   const uint16_t sbp_msg_type,
                                   const std::shared_ptr<sbp_state_t>& state,
                                   const std::string& topic)
    : SBPCallbackHandler(nh, sbp_msg_type, state), topic_(topic) {}

}  // namespace piksi_multi_cpp
