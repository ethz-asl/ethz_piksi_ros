#include <libsbp_ros_msgs/ros_conversion.h>
#include "piksi_multi_cpp/sbp_callback_handler/position_sampler.h"

namespace piksi_multi_cpp {

namespace lrm = libsbp_ros_msgs;
namespace gm = geometry_msgs;

void PositionSampler::callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) {
  // Advertise topic on first callback.
  if (!sampled_pos_pub_.has_value()) {
    sampled_pos_pub_ =
        nh_.advertise<piksi_rtk_msgs::PositionWithCovarianceStamped>(
            "ros/sampled_position", kQueueSize, kLatchTopic);
  }

  // Cast message.
  auto sbp_msg = (msg_pos_ecef_cov_t*)msg;
  if (!sbp_msg) {
    ROS_WARN("Cannot cast SBP message.");
    return;
  }

  // Convert to Eigen.
  Eigen::Vector3d position;
  lrm::convertCartesianPoint<msg_pos_ecef_cov_t>(*sbp_msg, &position);
  Eigen::Matrix3f cov;
  lrm::convertCartesianCov<msg_pos_ecef_cov_t>(*sbp_msg, &cov);
}

}  // namespace piksi_multi_cpp
