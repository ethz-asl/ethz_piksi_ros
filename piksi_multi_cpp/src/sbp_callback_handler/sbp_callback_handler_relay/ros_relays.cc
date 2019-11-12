#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_relays.h"

#include <libsbp_ros_msgs/ros_conversion.h>
#include <ros/assert.h>
#include <sensor_msgs/NavSatStatus.h>

namespace piksi_multi_cpp {

namespace lrm = libsbp_ros_msgs;
namespace gm = geometry_msgs;

bool RosPosEcefRelay::convertSbpMsgToRosMsg(const msg_pos_ecef_t& in,
                                            const uint8_t len,
                                            geometry_msgs::PointStamped* out) {
  ROS_ASSERT(out);
  lrm::convertCartesianPoint<msg_pos_ecef_t, gm::Point>(in, &(out->point));
  return true;
}

bool RosPosEcefCovRelay::convertSbpMsgToRosMsg(
    const msg_pos_ecef_cov_t& in, const uint8_t len,
    piksi_rtk_msgs::PositionWithCovarianceStamped* out) {
  ROS_ASSERT(out);
  lrm::convertCartesianPoint<msg_pos_ecef_cov_t, gm::Point>(
      in, &(out->position.position));
  lrm::convertCartesianCov<msg_pos_ecef_cov_t, boost::array<double, 9>>(
      in, &(out->position.covariance));
  return true;
}

bool RosPosLlhCovRelay::convertSbpMsgToRosMsg(const msg_pos_llh_cov_t& in,
                                              const uint8_t len,
                                              sensor_msgs::NavSatFix* out) {
  ROS_ASSERT(out);
  switch ((in.flags >> 0) & 0x7) {
    case 0:  // Invalid
    case 5:  // Dead reckoning
      out->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      break;
    case 1:  // SPP
      out->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      break;
    case 2:  // DGNSS
    case 3:  // Float RTK
    case 4:  // Fixed RTK
      out->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      break;
    case 6:  // SBAS
      out->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
      break;
    default:
      ROS_ERROR("Cannot infer fix status.");
      out->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      return false;
  }

  if (ros_receiver_state_.get()) {
    out->status.service = ros_receiver_state_->getNavSatServiceStatus();
  } else {
    return false;
  }

  lrm::convertWgs84Point<msg_pos_llh_cov_t, sensor_msgs::NavSatFix>(in, out);
  lrm::convertNedCov<msg_pos_llh_cov_t, boost::array<double, 9>>(
      in, &(out->position_covariance));
  out->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;

  return true;
}

bool RosBaselineNedRelay::convertSbpMsgToRosMsg(
    const msg_baseline_ned_t& in, const uint8_t len,
    piksi_rtk_msgs::PositionWithCovarianceStamped* out) {
  ROS_ASSERT(out);
  lrm::convertNedVector<msg_baseline_ned_t, gm::Point>(
      in, &(out->position.position));
  lrm::convertNedAccuracyToNedCov<msg_baseline_ned_t, boost::array<double, 9>>(
      in, &(out->position.covariance));
  return true;
}

bool RosVelEcefRelay::convertSbpMsgToRosMsg(const msg_vel_ecef_t& in,
                                            const uint8_t len,
                                            gm::Vector3Stamped* out) {
  ROS_ASSERT(out);
  lrm::convertCartesianVector<msg_vel_ecef_t, gm::Vector3>(in, &(out->vector));
  return true;
}

bool RosVelEcefCovRelay::convertSbpMsgToRosMsg(
    const msg_vel_ecef_cov_t& in, const uint8_t len,
    piksi_rtk_msgs::VelocityWithCovarianceStamped* out) {
  ROS_ASSERT(out);
  lrm::convertCartesianVector<msg_vel_ecef_cov_t, gm::Vector3>(
      in, &(out->velocity.velocity));
  lrm::convertCartesianCovMm<msg_vel_ecef_cov_t, boost::array<double, 9>>(
      in, &(out->velocity.covariance));
  return true;
}

bool RosVelNedRelay::convertSbpMsgToRosMsg(const msg_vel_ned_t& in,
                                           const uint8_t len,
                                           gm::Vector3Stamped* out) {
  ROS_ASSERT(out);
  lrm::convertNedVector<msg_vel_ned_t, gm::Vector3>(in, &(out->vector));
  return true;
}

bool RosVelNedCovRelay::convertSbpMsgToRosMsg(
    const msg_vel_ned_cov_t& in, const uint8_t len,
    piksi_rtk_msgs::VelocityWithCovarianceStamped* out) {
  ROS_ASSERT(out);
  lrm::convertNedVector<msg_vel_ned_cov_t, gm::Vector3>(
      in, &(out->velocity.velocity));
  lrm::convertNedCovMm<msg_vel_ned_cov_t, boost::array<double, 9>>(
      in, &(out->velocity.covariance));
  return true;
}

}  // namespace piksi_multi_cpp
