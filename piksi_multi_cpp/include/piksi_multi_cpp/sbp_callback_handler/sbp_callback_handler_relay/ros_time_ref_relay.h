#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_TIME_REF_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_TIME_REF_RELAY_H_

#include <libsbp/navigation.h>
#include <sensor_msgs/TimeReference.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class RosTimeReferenceRelay
    : public SBPCallbackHandlerRelay<msg_utc_time_t,
                                     sensor_msgs::TimeReference> {
 public:
  RosTimeReferenceRelay(const ros::NodeHandle& nh,
                        const std::shared_ptr<sbp_state_t>& state);

 private:
  bool convertSbpToRos(const msg_utc_time_t& sbp_msg, const uint8_t len,
                       sensor_msgs::TimeReference* ros_msg) override;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_TIME_REF_RELAY_H_
