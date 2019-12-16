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

bool RosPositionWithCovarianceEnuRelay::convertSbpMsgToRosMsg(
    const msg_pos_ecef_cov_t& in, const uint8_t len,
    piksi_rtk_msgs::PositionWithCovarianceStamped* out) {
  ROS_ASSERT(out);

  if (!convertEcefToEnu(in, &out->position.position)) return false;

  // Populate covariance.
  // Get rotation matrix.
  if (!geotf_handler_.get()) return false;
  Eigen::Vector3d x_enu_origin_wgs84;
  if (!geotf_handler_->getEnuOriginWgs84(&x_enu_origin_wgs84)) return false;

  Eigen::Matrix3d R_ENU_ECEF = libsbp_ros_msgs::getRotationEcefToEnu(
      x_enu_origin_wgs84.x(), x_enu_origin_wgs84.y());

  Eigen::Matrix3d cov_ecef;
  libsbp_ros_msgs::convertCartesianCov<msg_pos_ecef_cov_t>(in, &cov_ecef);
  // https://robotics.stackexchange.com/a/2557
  typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3dRow;
  Matrix3dRow cov_enu = R_ENU_ECEF * cov_ecef * R_ENU_ECEF.transpose();
  Matrix3dRow::Map(out->position.covariance.data()) = cov_enu;

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
  // Get rotation matrix.
  if (!geotf_handler_.get()) return false;
  Eigen::Vector3d x_enu_origin_wgs84;
  if (!geotf_handler_->getEnuOriginWgs84(&x_enu_origin_wgs84)) return false;

  Eigen::Matrix3d R_ENU_ECEF = libsbp_ros_msgs::getRotationEcefToEnu(
      x_enu_origin_wgs84.x(), x_enu_origin_wgs84.y());

  Eigen::Matrix3d cov_ecef;
  libsbp_ros_msgs::convertCartesianCov<msg_pos_ecef_cov_t>(in, &cov_ecef);

  // https://robotics.stackexchange.com/a/2557
  Eigen::Matrix3d cov_enu = R_ENU_ECEF * cov_ecef * R_ENU_ECEF.transpose();
  typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Matrix6dRow;
  Matrix6dRow cov = Matrix6dRow::Zero();
  cov.block<3, 3>(0, 0) = cov_enu;
  Matrix6dRow::Map(out->pose.covariance.data()) = cov;

  return true;
}

}  // namespace piksi_multi_cpp
