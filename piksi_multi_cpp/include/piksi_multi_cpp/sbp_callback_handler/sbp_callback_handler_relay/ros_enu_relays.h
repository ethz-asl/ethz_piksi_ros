#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_ENU_RELAYS_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_ENU_RELAYS_H_

#include <libsbp/navigation.h>
#include <ros/assert.h>
#include <Eigen/Dense>
#include <optional>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>

#include "piksi_multi_cpp/sbp_callback_handler/geotf_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/ros_time_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_relay.h"

#include <eigen_conversions/eigen_msg.h>
#include <libsbp_ros_msgs/ros_conversion.h>
#include <ros/assert.h>

namespace piksi_multi_cpp {

// Per default the ENU origin will be the base station position. But it can also
// be set to the current rover position or a user defined position.
template <class SbpMsgType, class RosMsgType>
class RosEnuRelay : public RosRelay<SbpMsgType, RosMsgType> {
 public:
  inline RosEnuRelay(const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
                     const std::shared_ptr<sbp_state_t>& state,
                     const std::string& topic,
                     const RosTimeHandler::Ptr& ros_time_handler,
                     const GeoTfHandler::Ptr& geotf_handler)
      : RosRelay<SbpMsgType, RosMsgType>(nh, sbp_msg_type, state, topic,
                                         ros_time_handler, "enu"),
        geotf_handler_(geotf_handler) {}

 protected:
  inline bool convertEcefToEnu(const SbpMsgType& in,
                               Eigen::Vector3d* x_enu) const {
    ROS_ASSERT(x_enu);

    // Convert position.
    Eigen::Vector3d x_ecef;
    libsbp_ros_msgs::convertCartesianPoint<SbpMsgType>(in, &x_ecef);

    if (!geotf_handler_.get()) return false;
    if (!geotf_handler_->getGeoTf().convertEcefToEnu(x_ecef, x_enu))
      return false;

    return true;
  }

  inline bool convertEcefToEnu(const SbpMsgType& in,
                               geometry_msgs::Point* out) {
    ROS_ASSERT(out)

    Eigen::Vector3d x_enu;
    if (!convertEcefToEnu(in, &x_enu)) return false;

    tf::pointEigenToMsg(x_enu, *out);
    return true;
  }

  inline bool convertEcefToEnu(const SbpMsgType& in,
                               geometry_msgs::Vector3* out) {
    ROS_ASSERT(out)

    Eigen::Vector3d x_enu;
    if (!convertEcefToEnu(in, &x_enu)) return false;

    tf::vectorEigenToMsg(x_enu, *out);
    return true;
  }

  GeoTfHandler::Ptr geotf_handler_;
};

class RosPosEnuRelay
    : public RosEnuRelay<msg_pos_ecef_t, geometry_msgs::PointStamped> {
 public:
  inline RosPosEnuRelay(const ros::NodeHandle& nh,
                        const std::shared_ptr<sbp_state_t>& state,
                        const RosTimeHandler::Ptr& ros_time_handler,
                        const GeoTfHandler::Ptr& geotf_handler)
      : RosEnuRelay(nh, SBP_MSG_POS_ECEF, state, "pos_enu", ros_time_handler,
                    geotf_handler) {}

 private:
  bool convertSbpMsgToRosMsg(const msg_pos_ecef_t& in, const uint8_t len,
                             geometry_msgs::PointStamped* out) override;
};

class RosTransformEnuRelay
    : public RosEnuRelay<msg_pos_ecef_t, geometry_msgs::TransformStamped> {
 public:
  inline RosTransformEnuRelay(const ros::NodeHandle& nh,
                              const std::shared_ptr<sbp_state_t>& state,
                              const RosTimeHandler::Ptr& ros_time_handler,
                              const GeoTfHandler::Ptr& geotf_handler)
      : RosEnuRelay(nh, SBP_MSG_POS_ECEF, state, "transform_enu",
                    ros_time_handler, geotf_handler) {}

 private:
  bool convertSbpMsgToRosMsg(const msg_pos_ecef_t& in, const uint8_t len,
                             geometry_msgs::TransformStamped* out) override;
};

class RosPositionWithCovarianceEnuRelay
    : public RosEnuRelay<msg_pos_ecef_cov_t,
                         piksi_rtk_msgs::PositionWithCovarianceStamped> {
 public:
  inline RosPositionWithCovarianceEnuRelay(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state,
      const RosTimeHandler::Ptr& ros_time_handler,
      const GeoTfHandler::Ptr& geotf_handler)
      : RosEnuRelay(nh, SBP_MSG_POS_ECEF_COV, state, "pos_enu_cov",
                    ros_time_handler, geotf_handler) {}

 private:
  bool convertSbpMsgToRosMsg(
      const msg_pos_ecef_cov_t& in, const uint8_t len,
      piksi_rtk_msgs::PositionWithCovarianceStamped* out) override;
};

class RosPoseWithCovarianceEnuRelay
    : public RosEnuRelay<msg_pos_ecef_cov_t,
                         geometry_msgs::PoseWithCovarianceStamped> {
 public:
  inline RosPoseWithCovarianceEnuRelay(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state,
      const RosTimeHandler::Ptr& ros_time_handler,
      const GeoTfHandler::Ptr& geotf_handler)
      : RosEnuRelay(nh, SBP_MSG_POS_ECEF_COV, state, "pose_enu_cov",
                    ros_time_handler, geotf_handler) {}

 private:
  bool convertSbpMsgToRosMsg(
      const msg_pos_ecef_cov_t& in, const uint8_t len,
      geometry_msgs::PoseWithCovarianceStamped* out) override;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_ENU_RELAYS_H_
