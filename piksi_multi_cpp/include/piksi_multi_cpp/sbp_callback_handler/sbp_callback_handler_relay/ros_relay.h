#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_RELAY_H_

#include <libsbp_ros_msgs/ros_conversion.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"
#include "piksi_multi_cpp/sbp_callback_handler/utc_time_buffer.h"

namespace piksi_multi_cpp {

// A base relay to publish ROS messages. Handles basic operations, e.g., time
// stamp conversion.
template <class SbpMsgType, class RosMsgType>
class RosRelay : public SBPCallbackHandlerRelay<SbpMsgType, RosMsgType> {
 public:
  inline RosRelay(const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
                  const std::shared_ptr<sbp_state_t>& state,
                  const std::string& topic,
                  const std::shared_ptr<UtcTimeBuffer>& utc_time_buffer,
                  const std::string& frame_id)
      : SBPCallbackHandlerRelay<SbpMsgType, RosMsgType>(nh, sbp_msg_type, state,
                                                        "ros/" + topic),
        utc_time_buffer_(utc_time_buffer),
        frame_id_(frame_id) {}

 private:
  // Implement this method to convert the message to the desired output type.
  virtual void convertSbpMsgToRosMsg(const SbpMsgType& sbp_msg,
                                     const uint8_t len,
                                     RosMsgType* ros_msg) = 0;

  inline RosMsgType convertSbpToRos(const SbpMsgType& sbp_msg,
                                    const uint8_t len) override {
    RosMsgType ros_msg;

    // Assign time stamp.
    if (utc_time_buffer_.get())
      ros_msg.header.stamp = utc_time_buffer_->getTime(sbp_msg.tow);
    else {
      ROS_WARN_ONCE("Using ros::Time::now() to stamp navigation data.");
      ros_msg.header.stamp = ros::Time::now();
    }

    // Set frame id.
    ros_msg.header.frame_id = frame_id_;

    // Manual conversion.
    convertSbpMsgToRosMsg(sbp_msg, len, &ros_msg);
    return ros_msg;
  }

  std::shared_ptr<UtcTimeBuffer> utc_time_buffer_;
  std::string frame_id_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_RELAY_H_
