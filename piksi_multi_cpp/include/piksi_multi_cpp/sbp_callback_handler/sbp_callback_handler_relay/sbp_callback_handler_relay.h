#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_H_

#include <ros/ros.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

namespace piksi_multi_cpp {

// This class handles all SBP messages and simply relays them to the ROS
// network. The concrete message relay needs to specifify the ROS message type,
// the SBP message class, set a topic name and implement the message conversion.
template <class ROSMsgType, class SBPMsgStruct>
class SBPCallbackHandlerRelay : public SBPCallbackHandler {
 public:
  // Registers a relay callback. There is a one to one mapping between
  // sbp_msg_type and publisher.
  inline SBPCallbackHandlerRelay(const ros::NodeHandle& nh,
                                 const uint16_t sbp_msg_type,
                                 const std::shared_ptr<sbp_state_t>& state,
                                 const std::string& topic)
      : SBPCallbackHandler(nh, sbp_msg_type, state) {
    // Advertise ROS topics.
    relay_pub_ = nh_.advertise<ROSMsgType>(topic, kQueueSize, kLatchTopic);
  }

 protected:
  // Concrete relays have to parse the struct into a ROS msg.
  virtual ROSMsgType convertSBPMsgToROSMsg(const SBPMsgStruct& sbp_msg) = 0;

 private:
  // Overwrites callback method to check number of subscribers, cast SBP message
  // and publish ROS msg.
  inline void callback(uint16_t sender_id, uint8_t len,
                       uint8_t msg[]) override {
    // Before doing anything check if anybody is listening.
    // https://answers.ros.org/question/197878/how-expensive-is-getnumsubscribers-of-publisher/
    if (relay_pub_.getNumSubscribers() == 0) return;

    // Cast message.
    auto sbp_msg = (SBPMsgStruct*)msg;
    if (!sbp_msg) {
      ROS_WARN("Cannot cast SBP message.");
      return;
    }

    // Convert SBP message.
    ROSMsgType ros_msg = convertSBPMsgToROSMsg(*sbp_msg);

    // Publish ROS msg.
    relay_pub_.publish(ros_msg);
  }

  // This publisher relays the incoming SBP message.
  ros::Publisher relay_pub_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_H_
