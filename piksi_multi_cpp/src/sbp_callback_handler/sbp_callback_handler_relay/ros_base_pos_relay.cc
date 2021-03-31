#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_base_pos_relay.h"

#include <libsbp_ros_msgs/ros_conversion.h>
#include <ros/assert.h>

namespace piksi_multi_cpp {

namespace lrm = libsbp_ros_msgs;
namespace gm = geometry_msgs;

RosBasePosEcefRelay::RosBasePosEcefRelay(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
    : SBPCallbackHandlerRelay<msg_base_pos_ecef_t, gm::PointStamped>(
          nh, SBP_MSG_BASE_POS_ECEF, state, "ros/base_pos_ecef") {}

bool RosBasePosEcefRelay::convertSbpToRos(const msg_base_pos_ecef_t& sbp_msg,
                                          const uint8_t len,
                                          gm::PointStamped* ros_msg) {
  ROS_ASSERT(mag);

  ros_msg->header.frame_id = "ecef";
  lrm::convertCartesianPoint<msg_base_pos_ecef_t, gm::Point>(sbp_msg,
                                                             &(ros_msg->point));

  return true;
}

}  // namespace piksi_multi_cpp
