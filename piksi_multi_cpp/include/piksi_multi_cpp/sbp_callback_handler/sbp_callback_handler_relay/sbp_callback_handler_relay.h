#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBP_CALLBACK_HANDLER_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBP_CALLBACK_HANDLER_RELAY_H_

#include <libsbp_ros_msgs/conversion.h>
#include <ros/ros.h>
#include <optional>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

namespace piksi_multi_cpp {

// This class handles all SBP messages and simply relays them to the ROS
// network. The concrete message relay needs to specifify the ROS message type,
// the SBP message class, set a topic name and implement the message conversion.
template <class SbpMsgType, class RosMsgType>
class SBPCallbackHandlerRelay : public SBPCallbackHandler {
 public:
  // Registers a relay callback. There is a one to one mapping between
  // sbp_msg_type and publisher.
  inline SBPCallbackHandlerRelay(const ros::NodeHandle& nh,
                                 const uint16_t sbp_msg_type,
                                 const std::shared_ptr<sbp_state_t>& state,
                                 const std::string& topic)
      : SBPCallbackHandler(sbp_msg_type, state), nh_(nh), topic_(topic) {}

 protected:
  // This publisher relays the incoming SBP message. It is generated and
  // advertised when callback is called for the first time, i.e., Piksi is
  // publishing this message.
  std::optional<ros::Publisher> relay_pub_;
  // A nodehandle with the correct ROS namespace.
  ros::NodeHandle nh_;

 private:
  //  Transforming incoming SBP message to ROS message.
  virtual bool convertSbpToRos(const SbpMsgType& sbp_msg, const uint8_t len,
                               RosMsgType* ros_msg) = 0;

  // Overwrites callback method to check number of subscribers, cast SBP message
  // and publish ROS msg.
  inline void callback(uint16_t sender_id, uint8_t len,
                       uint8_t msg[]) override {
    // Before doing anything check if anybody is listening.
    // https://answers.ros.org/question/197878/how-expensive-is-getnumsubscribers-of-publisher/
    if (relay_pub_.has_value() && relay_pub_.value().getNumSubscribers() == 0)
      return;
    // Advertise topic on first callback.
    if (!relay_pub_.has_value()) {
      relay_pub_ = nh_.advertise<RosMsgType>(topic_, kQueueSize, kLatchTopic);
    }

    // Cast message.
    auto sbp_msg = (SbpMsgType*)msg;
    if (!sbp_msg) {
      ROS_WARN("Cannot cast SBP message.");
      return;
    }

    // Convert and publish ROS msg.
    RosMsgType ros_msg;
    if (!convertSbpToRos(*sbp_msg, len, &ros_msg)) return;
    relay_pub_.value().publish(ros_msg);
  }
  std::string topic_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBP_CALLBACK_HANDLER_RELAY_H_
