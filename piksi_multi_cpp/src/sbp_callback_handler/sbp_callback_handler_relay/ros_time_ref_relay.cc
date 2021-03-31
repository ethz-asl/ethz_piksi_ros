#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_time_ref_relay.h"

#include <libsbp_ros_msgs/ros_conversion.h>
#include <ros/assert.h>

namespace piksi_multi_cpp {

namespace lrm = libsbp_ros_msgs;
namespace sm = sensor_msgs;

RosTimeReferenceRelay::RosTimeReferenceRelay(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
    : SBPCallbackHandlerRelay<msg_utc_time_t, sm::TimeReference>(
          nh, SBP_MSG_UTC_TIME, state, "ros/time_ref") {}

bool RosTimeReferenceRelay::convertSbpToRos(const msg_utc_time_t& sbp_msg,
                                            const uint8_t len,
                                            sm::TimeReference* ros_msg) {
  ROS_ASSERT(ros_msg);

  ros_msg->header.stamp = ros::Time::now();
  ros_msg->header.frame_id = "UTC";
  ros_msg->time_ref = lrm::convertUtcTimeToRosTime(sbp_msg);

  switch (sbp_msg.flags & 0b111) {
    case 0: {
      ros_msg->source = "invalid";
      break;
    }
    case 1: {
      ros_msg->source = "GNSS Solution";
      break;
    }
    case 2: {
      ros_msg->source = "Propagated";
      break;
    }
    default: {
      ros_msg->source = "unknown";
      break;
    }
  }

  return true;
}

}  // namespace piksi_multi_cpp
