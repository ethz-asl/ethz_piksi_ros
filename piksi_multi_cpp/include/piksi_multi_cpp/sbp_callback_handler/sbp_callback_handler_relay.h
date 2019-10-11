#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_H_

#include <libsbp/sbp.h>
#include <ros/ros.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

namespace piksi_multi_cpp {

// This class handles all SBP messages and simply relays them to the ROS
// network.
class SBPCallbackHandlerRelay : public SBPCallbackHandler {
 public:
  // Registers a relay callback. There is a one to one mapping between
  // sbp_msg_type and publisher.
  SBPCallbackHandlerRelay(const ros::NodeHandle& nh,
                          const uint16_t sbp_msg_type,
                          const std::shared_ptr<sbp_state_t>& state);

 protected:
  // This publisher relays the incoming SBP message.
  ros::Publisher relay_pub_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_H_
