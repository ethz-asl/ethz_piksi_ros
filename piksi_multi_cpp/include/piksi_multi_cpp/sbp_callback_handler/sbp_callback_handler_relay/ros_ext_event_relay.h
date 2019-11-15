#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_EXT_EVENT_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_EXT_EVENT_RELAY_H_

#include <libsbp/ext_events.h>
#include <piksi_rtk_msgs/ExtEvent.h>
#include <ros/assert.h>
#include "piksi_multi_cpp/sbp_callback_handler/ros_time_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class RosExtEventRelay
    : public SBPCallbackHandlerRelay<msg_ext_event_t,
                                     piksi_rtk_msgs::ExtEvent> {
 public:
  inline RosExtEventRelay(const ros::NodeHandle& nh,
                          const std::shared_ptr<sbp_state_t>& state,
                          const RosTimeHandler::Ptr& ros_time_handler)
      : SBPCallbackHandlerRelay<msg_ext_event_t, piksi_rtk_msgs::ExtEvent>(
            nh, SBP_MSG_EXT_EVENT, state, "ros/ext_event"),
        ros_time_handler_(ros_time_handler) {}

 private:
  inline bool convertSbpToRos(const msg_ext_event_t& sbp_msg, const uint8_t len,
                              piksi_rtk_msgs::ExtEvent* ros_msg) override {
    ROS_ASSERT(ros_msg);
    if (!ros_time_handler_.get()) {
      ROS_ERROR("No time handler set.");
      return false;
    }

    // Create ROS data.
    ros_msg->stamp.data = ros_time_handler_->convertGpsTime(
        sbp_msg.wn, sbp_msg.tow, sbp_msg.ns_residual);
    ros_msg->pin_value = (sbp_msg.flags >> 0) & 0x1;
    ros_msg->quality = (sbp_msg.flags >> 1) & 0x1;
    ros_msg->pin = sbp_msg.pin;

    return true;
  }

  RosTimeHandler::Ptr ros_time_handler_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_EXT_EVENT_RELAY_H_
