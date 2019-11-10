#include <eigen_conversions/eigen_msg.h>
#include <libsbp_ros_msgs/ros_conversion.h>
#include "piksi_multi_cpp/sbp_callback_handler/position_sampler.h"

namespace piksi_multi_cpp {

namespace lrm = libsbp_ros_msgs;
namespace prm = piksi_rtk_msgs;

void PositionSampler::callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) {
  if (!num_desired_fixes_.has_value()) return;
  if (num_fixes_ >= num_desired_fixes_.value()) return;
  if (!ros_time_handler_.get()) {
    ROS_ERROR("No time handler set.");
    return;
  }

  // Set least square variables.
  if (!y_.has_value()) {
    y_ = Eigen::VectorXd(3 * num_desired_fixes_.value());
  }
  if (!R_inv_.has_value()) {
    R_inv_ = Eigen::MatrixXd::Zero(3 * num_desired_fixes_.value(),
                                   3 * num_desired_fixes_.value());
  }

  // Advertise topic on first callback.
  if (!final_pos_pub_.has_value()) {
    final_pos_pub_ =
        nh_.advertise<piksi_rtk_msgs::PositionWithCovarianceStamped>(
            "ros/final_sampled_position", kQueueSize, kLatchTopic);
  }
  if (!current_pos_pub_.has_value()) {
    current_pos_pub_ =
        nh_.advertise<piksi_rtk_msgs::PositionWithCovarianceStamped>(
            "ros/current_sampled_position", kQueueSize, kLatchTopic);
  }

  // Cast message.
  auto sbp_msg = (msg_pos_ecef_cov_t*)msg;
  if (!sbp_msg) {
    ROS_WARN("Cannot cast SBP message.");
    return;
  }

  // Convert measurement to Eigen.
  Eigen::Vector3d z;
  lrm::convertCartesianPoint<msg_pos_ecef_cov_t>(*sbp_msg, &z);
  Eigen::Matrix3d R;
  lrm::convertCartesianCov<msg_pos_ecef_cov_t>(*sbp_msg, &R);

  // Cache least square measurement values.
  size_t block_idx = 3 * num_fixes_++;
  y_.value().segment(block_idx, 3) = z;
  R_inv_.value().block<3, 3>(block_idx, block_idx) = R.inverse();

  if (x_.has_value() && P_.has_value()) {
    // Measurement update.
    // Innovation.
    auto y = z - x_.value();
    auto S = P_.value() + R;
    // Gain.
    auto K = P_.value() * S.inverse();
    // Update.
    x_.value() += K * y;
    P_.value() = (Eigen::Matrix3d::Identity() - K) * P_.value();
  } else {
    // Initialize.
    x_ = z;
    P_ = R;
  }

  // Logging.
  ROS_INFO_STREAM("Measurement: "
                  << z.transpose() << "; 3-sigma bound: "
                  << R.eigenvalues().real().cwiseSqrt().transpose()
                  << "; temporary mean: " << x_.value().transpose()
                  << "; 3-sigma bound: "
                  << P_.value().eigenvalues().real().cwiseSqrt().transpose());

  prm::PositionWithCovarianceStamped current_pos;
  tf::pointEigenToMsg(x_.value(), current_pos.position.position);
  typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3dRow;
  Matrix3dRow::Map(current_pos.position.covariance.data()) = P_.value();
  current_pos.header.frame_id = "ecef";
  current_pos.header.stamp = ros_time_handler_->lookupTime(sbp_msg->tow);
  current_pos_pub_.value().publish(current_pos);

  // Compute final least squares solution.
  if (num_fixes_ >= num_desired_fixes_.value()) {
  //  auto H =
  //      Eigen::Matrix3d::Identity().replicate<num_desired_fixes_.value(), 1>();
    // auto a =
  }
}

}  // namespace piksi_multi_cpp
