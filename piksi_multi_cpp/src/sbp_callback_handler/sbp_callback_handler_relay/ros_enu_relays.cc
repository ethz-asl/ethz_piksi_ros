#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_enu_relays.h"

namespace piksi_multi_cpp {

bool RosPosEnuRelay::convertSbpMsgToRosMsg(const msg_pos_ecef_t& in,
                                           const uint8_t len,
                                           geometry_msgs::PointStamped* out) {
  ROS_ASSERT(out);

  return convertEcefToEnu(in, &out->point);
}

bool RosTransformEnuRelay::convertSbpMsgToRosMsg(
    const msg_pos_ecef_t& in, const uint8_t len,
    geometry_msgs::TransformStamped* out) {
  ROS_ASSERT(out);

  if (!convertEcefToEnu(in, &out->transform.translation)) return false;

  // TODO(rikba): Also add orientation information if available.
  out->transform.rotation.w = 1.0;

  return true;
}

bool RosPoseWithCovarianceEnuRelay::convertSbpMsgToRosMsg(
    const msg_pos_ecef_cov_t& in, const uint8_t len,
    geometry_msgs::PoseWithCovarianceStamped* out) {
  ROS_ASSERT(out);

  if (!convertEcefToEnu(in, &out->pose.pose.position)) return false;

  // TODO(rikba): Also add orientation information if available.
  out->pose.pose.orientation.w = 1.0;

  // Populate covariance.
  // TODO(rikba): Properly rotate ECEF to ENU!! This is bullshit and may only
  // work if you live in Greenwich.
  typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Matrix6dRow;
  Matrix6dRow cov = Matrix6dRow::Zero();
  cov(0, 0) = in.cov_y_y;
  cov(1, 1) = in.cov_y_y;
  cov(2, 2) = in.cov_x_x;
  Matrix6dRow::Map(out->pose.covariance.data()) = cov;

  return true;
}

}  // namespace piksi_multi_cpp
