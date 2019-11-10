#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_POSITION_SAMPLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_POSITION_SAMPLER_H_

#include <libsbp/navigation.h>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include <ros/ros.h>
#include <optional>
#include "piksi_multi_cpp/sbp_callback_handler/ros_time_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

namespace piksi_multi_cpp {

class PositionSampler : public SBPCallbackHandler {
 public:
  inline PositionSampler(const ros::NodeHandle& nh,
                         const std::shared_ptr<sbp_state_t>& state,
                         const RosTimeHandler::Ptr& ros_time_handler)
      : SBPCallbackHandler(SBP_MSG_POS_ECEF_COV, state),
        nh_(nh),
        ros_time_handler_(ros_time_handler) {}

 private:
  void callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) override;
  void publishPosition(const ros::Publisher& pub, const Eigen::Vector3d& x,
                       const Eigen::Matrix3d& cov, const uint32_t tow) const;

  ros::NodeHandle nh_;
  RosTimeHandler::Ptr ros_time_handler_;
  std::optional<ros::Publisher> ml_pos_pub_;
  std::optional<ros::Publisher> kf_pos_pub_;

  // Sampler state.
  std::optional<uint32_t> num_desired_fixes_;
  uint32_t num_fixes_ = 0;

  // Kalman filter state.
  std::optional<Eigen::Vector3d> x_;
  std::optional<Eigen::Matrix3d> P_;

  // Least square variables.
  std::optional<Eigen::VectorXd> y_;
  std::optional<Eigen::MatrixXd> R_inv_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_POSITION_SAMPLER_H_
