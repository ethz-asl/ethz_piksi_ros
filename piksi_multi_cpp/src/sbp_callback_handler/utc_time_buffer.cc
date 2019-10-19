#include "piksi_multi_cpp/sbp_callback_handler/utc_time_buffer.h"

#include <libsbp/navigation.h>
#include <libsbp_ros_msgs/ros_conversion.h>

namespace piksi_multi_cpp {

UtcTimeBuffer::UtcTimeBuffer(const ros::NodeHandle& nh,
                             const std::shared_ptr<sbp_state_t>& state)
    : SBPCallbackHandler(nh, SBP_MSG_UTC_TIME, state) {}

ros::Time UtcTimeBuffer::getTime(const uint32_t tow) {
  if (time_map_.find(tow) != time_map_.end()) {
    return time_map_[tow];
  } else {
    ROS_WARN(
        "Timestamp only ms precision. Did not receive UTC time for tow %d.",
        tow);
    return libsbp_ros_msgs::convertTowToRosTime(tow);
  }
}

void UtcTimeBuffer::callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) {
  // Cast message.
  auto sbp_msg = (msg_utc_time_t*)msg;
  if (!sbp_msg) {
    ROS_WARN("Cannot cast SBP message.");
    return;
  }

  time_map_[sbp_msg->tow] = libsbp_ros_msgs::convertUtcTimeToRosTime(*sbp_msg);

  // Buffer book keeping.
  while (time_map_.size() > buffer_size_) {
    auto it = time_map_.begin();
    time_map_.erase(it);
  }
}

}  // namespace piksi_multi_cpp
