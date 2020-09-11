#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_POSITION_SAMPLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_POSITION_SAMPLER_H_

#include <libsbp/navigation.h>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include <piksi_rtk_msgs/SamplePosition.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include "piksi_multi_cpp/sbp_callback_handler/geotf_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/ros_time_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

namespace piksi_multi_cpp {

class PositionSampler : public SBPCallbackHandler {
 public:
  typedef std::shared_ptr<PositionSampler> Ptr;
  PositionSampler(const ros::NodeHandle& nh,
                  const std::shared_ptr<sbp_state_t>& state,
                  const RosTimeHandler::Ptr& ros_time_handler,
                  const GeoTfHandler::Ptr& geotf_handler);

  bool startSampling(const uint32_t num_desired_fixes,
                     const std::string& file = "", bool set_enu = false,
                     double offset_z = 0.0);
  inline bool isSampling() { return !x_ml_.has_value(); }
  bool getResult(Eigen::Vector3d* x_ecef, Eigen::Matrix3d* cov);

 private:
  bool samplePositionCallback(piksi_rtk_msgs::SamplePosition::Request& req,
                              piksi_rtk_msgs::SamplePosition::Response& res);
  void callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) override;
  void publishPosition(const ros::Publisher& pub, const Eigen::Vector3d& x,
                       const Eigen::Matrix3d& cov, const uint32_t tow) const;
  void publishProgress();
  bool savePositionToFile(const Eigen::Vector3d& x, const Eigen::Matrix3d& cov,
                          const uint32_t num_fixes) const;
  std::string getTimeStr() const;

  ros::NodeHandle nh_;
  RosTimeHandler::Ptr ros_time_handler_;
  GeoTfHandler::Ptr geotf_handler_;
  std::optional<ros::Publisher> ml_pos_pub_;
  std::optional<ros::Publisher> kf_pos_pub_;
  std::optional<ros::Publisher> info_pub_;
  ros::ServiceServer sample_pos_srv_;

  // Sampler state.
  std::optional<uint32_t> num_desired_fixes_;
  uint32_t num_fixes_ = 0;
  std::string file_;
  bool set_enu_ = false;
  double offset_z_ = 0.0;

  // Kalman filter state.
  std::optional<Eigen::Vector3d> x_;
  std::optional<Eigen::Matrix3d> P_;

  // Least square variables.
  std::optional<Eigen::VectorXd> y_;
  std::optional<Eigen::MatrixXd> R_inv_;
  std::optional<Eigen::Vector3d> x_ml_;
  std::optional<Eigen::Matrix3d> P_ml_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_POSITION_SAMPLER_H_
