#include <eigen_conversions/eigen_msg.h>
#include <libsbp_ros_msgs/ros_conversion.h>
#include <ros/assert.h>
#include <Eigen/Dense>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include "piksi_multi_cpp/sbp_callback_handler/position_sampler.h"

namespace piksi_multi_cpp {

namespace lrm = libsbp_ros_msgs;
namespace prm = piksi_rtk_msgs;
namespace fs = std::experimental::filesystem;

PositionSampler::PositionSampler(const ros::NodeHandle& nh,
                                 const std::shared_ptr<sbp_state_t>& state,
                                 const RosTimeHandler::Ptr& ros_time_handler)
    : SBPCallbackHandler(SBP_MSG_POS_ECEF_COV, state),
      nh_(nh),
      ros_time_handler_(ros_time_handler) {
  sample_pos_srv_ = nh_.advertiseService(
      "sample_position", &PositionSampler::samplePositionCallback, this);
}

void PositionSampler::startSampling(const uint32_t num_desired_fixes,
                                    const std::string& file) {
  num_desired_fixes_ = num_desired_fixes;
  num_fixes_ = 0;
  file_ = file;
  x_.reset();
  P_.reset();
  y_.reset();
  R_inv_.reset();
  x_ml_.reset();
  P_ml_.reset();
}

bool PositionSampler::getResult(Eigen::Vector3d* x_ecef, Eigen::Matrix3d* cov) {
  ROS_ASSERT(x_ecef);
  ROS_ASSERT(cov);

  if (isSampling()) return false;

  *x_ecef = x_ml_.value();
  *cov = P_ml_.value();

  return true;
}

bool PositionSampler::samplePositionCallback(
    piksi_rtk_msgs::SamplePosition::Request& req,
    piksi_rtk_msgs::SamplePosition::Response& res) {
  startSampling(req.num_desired_fixes, req.file);
  return true;
}

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
  if (!ml_pos_pub_.has_value()) {
    ml_pos_pub_ = nh_.advertise<piksi_rtk_msgs::PositionWithCovarianceStamped>(
        "position_sampler/ml_position", kQueueSize, kLatchTopic);
  }
  if (!kf_pos_pub_.has_value()) {
    kf_pos_pub_ = nh_.advertise<piksi_rtk_msgs::PositionWithCovarianceStamped>(
        "position_sampler/kf_position", kQueueSize, kLatchTopic);
  }

  // Cast message.
  auto sbp_msg = (msg_pos_ecef_cov_t*)msg;
  if (!sbp_msg) {
    ROS_WARN("Cannot cast SBP message.");
    return;
  }

  // Check if fix mode is valid.
  if (((sbp_msg->flags >> 0) && 0x7) == 0) {
    ROS_WARN_THROTTLE(5, "Cannot sample position. Fix mode invalid.");
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

  // Kalman filter measurement update.
  if (x_.has_value() && P_.has_value()) {
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
  ROS_DEBUG_STREAM("Measurement: "
                   << z.transpose() << "; 3-sigma bound: "
                   << R.eigenvalues().real().cwiseSqrt().transpose()
                   << "; temporary mean: " << x_.value().transpose()
                   << "; 3-sigma bound: "
                   << P_.value().eigenvalues().real().cwiseSqrt().transpose());
  publishPosition(kf_pos_pub_.value(), x_.value(), P_.value(), sbp_msg->tow);

  // Compute final least squares solution.
  if (num_fixes_ >= num_desired_fixes_.value()) {
    auto H =
        Eigen::Matrix3d::Identity().replicate(num_desired_fixes_.value(), 1);
    auto A = H.transpose() * R_inv_.value() * H;
    auto b = H.transpose() * R_inv_.value() * y_.value();
    auto dec = A.colPivHouseholderQr();
    x_ml_ = dec.solve(b);
    P_ml_ = A.inverse();
    ROS_INFO_STREAM(
        "ML estimate: "
        << x_ml_.value().transpose() << "; 3-sigma bound: "
        << P_ml_.value().eigenvalues().real().cwiseSqrt().transpose());
    publishPosition(ml_pos_pub_.value(), x_ml_.value(), P_ml_.value(),
                    sbp_msg->tow);
    ROS_ERROR_COND(
        !savePositionToFile(x_ml_.value(), P_ml_.value(), num_fixes_),
        "Failed to save position to file.");
  }

  // Logging.
  ROS_INFO_THROTTLE(5.0, "Sampling position %d/%d", num_fixes_,
                    num_desired_fixes_.value());
}

void PositionSampler::publishPosition(const ros::Publisher& pub,
                                      const Eigen::Vector3d& x,
                                      const Eigen::Matrix3d& cov,
                                      const uint32_t tow) const {
  prm::PositionWithCovarianceStamped pos;
  tf::pointEigenToMsg(x, pos.position.position);
  typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3dRow;
  Matrix3dRow::Map(pos.position.covariance.data()) = cov;
  pos.header.frame_id = "ecef";
  pos.header.stamp = ros_time_handler_->lookupTime(tow);
  pub.publish(pos);
}

std::string PositionSampler::getTimeStr() const {
  std::time_t now =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  char buffer[80];
  std::strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", std::localtime(&now));
  return std::string(buffer);
}

bool PositionSampler::savePositionToFile(const Eigen::Vector3d& x,
                                         const Eigen::Matrix3d& cov,
                                         const uint32_t num_fixes) const {
  std::string file = file_;
  if (file.empty()) {
    // Save to default path.
    file = std::string(std::getenv("HOME")) + "/.ros/position_samples/";
    file += getTimeStr();
    file += "_sampled_position_";
    file += nh_.getUnresolvedNamespace();
    file += "_" + std::to_string(num_fixes);
    file += ".txt";
  }
  ROS_INFO("Saving sampled position to %s", file.c_str());

  // Create directory recursively.
  fs::path path = file;
  if (!fs::exists(path.parent_path())) {
    if (!fs::create_directories(path.parent_path())) return false;
  }

  std::fstream fs;
  fs.open(file, std::fstream::out);
  if (!fs.is_open()) return false;
  fs << "x_ecef: " << boost::lexical_cast<std::string>(x.x()) << std::endl;
  fs << "y_ecef: " << boost::lexical_cast<std::string>(x.y()) << std::endl;
  fs << "z_ecef: " << boost::lexical_cast<std::string>(x.z()) << std::endl;
  fs << "cov_x_x_ecef: " << boost::lexical_cast<std::string>(cov(0, 0))
     << std::endl;
  fs << "cov_x_y_ecef: " << boost::lexical_cast<std::string>(cov(0, 1))
     << std::endl;
  fs << "cov_x_z_ecef: " << boost::lexical_cast<std::string>(cov(0, 2))
     << std::endl;
  fs << "cov_y_y_ecef: " << boost::lexical_cast<std::string>(cov(1, 1))
     << std::endl;
  fs << "cov_y_z_ecef: " << boost::lexical_cast<std::string>(cov(1, 2))
     << std::endl;
  fs << "cov_z_z_ecef: " << boost::lexical_cast<std::string>(cov(2, 2))
     << std::endl;
  fs.close();

  return true;
}

}  // namespace piksi_multi_cpp
