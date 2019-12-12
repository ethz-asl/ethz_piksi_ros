#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_NED_RELAYS_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_NED_RELAYS_H_

#include <geotf/geodetic_converter.h>
#include <libsbp/navigation.h>
#include <ros/assert.h>
#include <Eigen/Dense>
#include <optional>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
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
class RosNedRelay : public RosRelay<SbpMsgType, RosMsgType> {
 public:
  inline RosNedRelay(const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
                     const std::shared_ptr<sbp_state_t>& state,
                     const std::string& topic,
                     const RosTimeHandler::Ptr& ros_time_handler,
                     const GeoTfHandler::Ptr& geotf_handler)
      : RosRelay<SbpMsgType, RosMsgType>(nh, sbp_msg_type, state, topic,
                                         ros_time_handler, "ned"),
        geotf_handler_(geotf_handler) {}

 protected:
  inline bool convertEcefToNed(const SbpMsgType& in,
                               Eigen::Vector3d* x_ned) const {
    ROS_ASSERT(x_ned);

    // Convert position.
    Eigen::Vector3d x_ecef, x_enu;
    libsbp_ros_msgs::convertCartesianPoint<msg_pos_ecef_t>(in, &x_ecef);

    if (!geotf_handler_.get()) return false;
    if (!geotf_handler_->getGeoTf().convert("ecef", x_ecef, "enu", &x_enu))
      return false;
    x_ned->x() = x_enu.y();
    x_ned->y() = x_enu.x();
    x_ned->z() = -x_enu.z();

    return true;
  }

  inline bool convertEcefToNed(const SbpMsgType& in,
                               geometry_msgs::Point* out) {
    ROS_ASSERT(out)

    Eigen::Vector3d x_ned;
    if (!convertEcefToNed(in, &x_ned)) return false;

    tf::pointEigenToMsg(x_ned, *out);
    return true;
  }

  inline bool convertEcefToNed(const SbpMsgType& in,
                               geometry_msgs::Vector3* out) {
    ROS_ASSERT(out)

    Eigen::Vector3d x_ned;
    if (!convertEcefToNed(in, &x_ned)) return false;

    tf::vectorEigenToMsg(x_ned, *out);
    return true;
  }

  GeoTfHandler::Ptr geotf_handler_;
};

class RosPosNedRelay
    : public RosNedRelay<msg_pos_ecef_t, geometry_msgs::PointStamped> {
 public:
  inline RosPosNedRelay(const ros::NodeHandle& nh,
                        const std::shared_ptr<sbp_state_t>& state,
                        const RosTimeHandler::Ptr& ros_time_handler,
                        const GeoTfHandler::Ptr& geotf_handler)
      : RosNedRelay(nh, SBP_MSG_POS_ECEF, state, "pos_ned", ros_time_handler,
                    geotf_handler) {}

 private:
  bool convertSbpMsgToRosMsg(const msg_pos_ecef_t& in, const uint8_t len,
                             geometry_msgs::PointStamped* out) override;
};

class RosTransformNedRelay
    : public RosNedRelay<msg_pos_ecef_t, geometry_msgs::TransformStamped> {
 public:
  inline RosTransformNedRelay(const ros::NodeHandle& nh,
                              const std::shared_ptr<sbp_state_t>& state,
                              const RosTimeHandler::Ptr& ros_time_handler,
                              const GeoTfHandler::Ptr& geotf_handler)
      : RosNedRelay(nh, SBP_MSG_POS_ECEF, state, "transform_ned",
                    ros_time_handler, geotf_handler) {}

 private:
  bool convertSbpMsgToRosMsg(const msg_pos_ecef_t& in, const uint8_t len,
                             geometry_msgs::TransformStamped* out) override;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_NED_RELAYS_H_
