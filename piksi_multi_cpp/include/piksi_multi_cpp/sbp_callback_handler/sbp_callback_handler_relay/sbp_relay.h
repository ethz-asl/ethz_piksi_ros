#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBP_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBP_RELAY_H_

#include <libsbp_ros_msgs/conversion.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

// A base relay to publish ROS messages. Handles basic operations, e.g., time
// stamp conversion.
template <class SbpMsgType, class RosMsgType>
class SbpRelay : public SBPCallbackHandlerRelay<SbpMsgType, RosMsgType> {
 public:
  inline SbpRelay(const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
                  const std::shared_ptr<sbp_state_t>& state,
                  const std::string& topic)
      : SBPCallbackHandlerRelay<SbpMsgType, RosMsgType>(nh, sbp_msg_type, state,
                                                        "sbp/" + topic) {}

 private:
  inline bool convertSbpToRos(const SbpMsgType& sbp_msg, const uint8_t len,
                              RosMsgType* ros_msg) override {
    *ros_msg = libsbp_ros_msgs::convertSbpMsgToRosMsg(sbp_msg, len);
    return true;
  }
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBP_RELAY_H_
