#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_RELAYS_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_RELAYS_H_

#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_receiver_state.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_relay.h"

#include <optional>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include <piksi_rtk_msgs/VelocityWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <libsbp/navigation.h>

#include <libsbp_ros_msgs/conversion.h>
#include <ros/ros.h>

// This file contains specific implementations of navigation relays, i.e.,
// position, baseline, and velocity publishing.

namespace piksi_multi_cpp {

class RosPosEcefRelay
    : public RosRelay<msg_pos_ecef_t, geometry_msgs::PointStamped> {
 public:
  inline RosPosEcefRelay(const ros::NodeHandle& nh,
                         const std::shared_ptr<sbp_state_t>& state,
                         const RosTimeHandler::Ptr& ros_time_handler)
      : RosRelay(nh, SBP_MSG_POS_ECEF, state, "pos_ecef", ros_time_handler,
                 "ecef") {}

 private:
  bool convertSbpMsgToRosMsg(const msg_pos_ecef_t& in, const uint8_t len,
                             geometry_msgs::PointStamped* out) override;
};

class RosPosEcefCovRelay
    : public RosRelay<msg_pos_ecef_cov_t,
                      piksi_rtk_msgs::PositionWithCovarianceStamped> {
 public:
  inline RosPosEcefCovRelay(const ros::NodeHandle& nh,
                            const std::shared_ptr<sbp_state_t>& state,
                            const RosTimeHandler::Ptr& ros_time_handler)
      : RosRelay(nh, SBP_MSG_POS_ECEF_COV, state, "pos_ecef_cov",
                 ros_time_handler, "ecef") {}

 private:
  bool convertSbpMsgToRosMsg(
      const msg_pos_ecef_cov_t& in, const uint8_t len,
      piksi_rtk_msgs::PositionWithCovarianceStamped* out) override;
};

class RosPosLlhCovRelay
    : public RosRelay<msg_pos_llh_cov_t, sensor_msgs::NavSatFix> {
 public:
  inline RosPosLlhCovRelay(const ros::NodeHandle& nh,
                           const std::shared_ptr<sbp_state_t>& state,
                           const RosTimeHandler::Ptr& ros_time_handler,
                           const RosReceiverState::Ptr& ros_receiver_state)
      : RosRelay(nh, SBP_MSG_POS_LLH_COV, state, "navsatfix", ros_time_handler,
                 "wgs84"),
        ros_receiver_state_(ros_receiver_state) {}

 private:
  bool convertSbpMsgToRosMsg(const msg_pos_llh_cov_t& in, const uint8_t len,
                             sensor_msgs::NavSatFix* out) override;
  RosReceiverState::Ptr ros_receiver_state_;
};

class RosBaselineNedRelay
    : public RosRelay<msg_baseline_ned_t,
                      piksi_rtk_msgs::PositionWithCovarianceStamped> {
 public:
  inline RosBaselineNedRelay(const ros::NodeHandle& nh,
                             const std::shared_ptr<sbp_state_t>& state,
                             const RosTimeHandler::Ptr& ros_time_handler)
      : RosRelay(nh, SBP_MSG_BASELINE_NED, state, "baseline_ned",
                 ros_time_handler, "ned_base_station") {}

 private:
  bool convertSbpMsgToRosMsg(
      const msg_baseline_ned_t& in, const uint8_t len,
      piksi_rtk_msgs::PositionWithCovarianceStamped* out) override;
};

class RosVelEcefRelay
    : public RosRelay<msg_vel_ecef_t, geometry_msgs::Vector3Stamped> {
 public:
  inline RosVelEcefRelay(const ros::NodeHandle& nh,
                         const std::shared_ptr<sbp_state_t>& state,
                         const RosTimeHandler::Ptr& ros_time_handler)
      : RosRelay(nh, SBP_MSG_VEL_ECEF, state, "vel_ecef", ros_time_handler,
                 "ecef") {}

 private:
  bool convertSbpMsgToRosMsg(const msg_vel_ecef_t& in, const uint8_t len,
                             geometry_msgs::Vector3Stamped* out) override;
};

class RosVelEcefCovRelay
    : public RosRelay<msg_vel_ecef_cov_t,
                      piksi_rtk_msgs::VelocityWithCovarianceStamped> {
 public:
  inline RosVelEcefCovRelay(const ros::NodeHandle& nh,
                            const std::shared_ptr<sbp_state_t>& state,
                            const RosTimeHandler::Ptr& ros_time_handler)
      : RosRelay(nh, SBP_MSG_VEL_ECEF_COV, state, "vel_ecef_cov",
                 ros_time_handler, "ecef") {}

 private:
  bool convertSbpMsgToRosMsg(
      const msg_vel_ecef_cov_t& in, const uint8_t len,
      piksi_rtk_msgs::VelocityWithCovarianceStamped* out) override;
};

class RosVelNedRelay
    : public RosRelay<msg_vel_ned_t, geometry_msgs::Vector3Stamped> {
 public:
  inline RosVelNedRelay(const ros::NodeHandle& nh,
                        const std::shared_ptr<sbp_state_t>& state,
                        const RosTimeHandler::Ptr& ros_time_handler)
      : RosRelay(nh, SBP_MSG_VEL_NED, state, "vel_ned", ros_time_handler,
                 "ned") {}

 private:
  bool convertSbpMsgToRosMsg(const msg_vel_ned_t& in, const uint8_t len,
                             geometry_msgs::Vector3Stamped* out) override;
};

class RosVelNedCovRelay
    : public RosRelay<msg_vel_ned_cov_t,
                      piksi_rtk_msgs::VelocityWithCovarianceStamped> {
 public:
  inline RosVelNedCovRelay(const ros::NodeHandle& nh,
                           const std::shared_ptr<sbp_state_t>& state,
                           const RosTimeHandler::Ptr& ros_time_handler)
      : RosRelay(nh, SBP_MSG_VEL_NED_COV, state, "vel_ned_cov",
                 ros_time_handler, "ned") {}

 private:
  bool convertSbpMsgToRosMsg(
      const msg_vel_ned_cov_t& in, const uint8_t len,
      piksi_rtk_msgs::VelocityWithCovarianceStamped* out) override;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_RELAYS_H_
