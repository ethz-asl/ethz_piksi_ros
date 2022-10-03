#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_base_pos_relay.h"

#include <libsbp_ros_msgs/ros_conversion.h>
#include <ros/assert.h>

namespace piksi_multi_cpp {

namespace lrm = libsbp_ros_msgs;
namespace gm = geometry_msgs;
namespace sm = sensor_msgs;

RosBasePosEcefRelay::RosBasePosEcefRelay(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
    : SBPCallbackHandlerRelay<msg_base_pos_ecef_t, gm::PointStamped>(
          nh, SBP_MSG_BASE_POS_ECEF, state, "ros/base_pos_ecef") {}

bool RosBasePosEcefRelay::convertSbpToRos(const msg_base_pos_ecef_t& sbp_msg,
                                          const uint8_t len,
                                          gm::PointStamped* ros_msg) {
  ROS_ASSERT(ros_msg);

  ros_msg->header.frame_id = "ecef";
  lrm::convertCartesianPoint<msg_base_pos_ecef_t, gm::Point>(sbp_msg,
                                                             &(ros_msg->point));

  return true;
}

RosBasePosLlhRelay::RosBasePosLlhRelay(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state,
    const GeoTfHandler::Ptr& geotf_handler)
    : SBPCallbackHandlerRelay<msg_base_pos_ecef_t, sm::NavSatFix>(
          nh, SBP_MSG_BASE_POS_ECEF, state, "ros/base_pos_navsatfix"),
      geotf_handler_(geotf_handler) {}

bool RosBasePosLlhRelay::convertSbpToRos(const msg_base_pos_ecef_t& sbp_msg,
                                         const uint8_t len,
                                         sm::NavSatFix* ros_msg) {
  ROS_ASSERT(ros_msg);

  ros_msg->header.frame_id = "wgs84";
  ros_msg->status.status = sm::NavSatStatus::STATUS_SBAS_FIX;
  ros_msg->status.service = sm::NavSatStatus::SERVICE_GPS;
  ros_msg->status.service |= sm::NavSatStatus::SERVICE_GLONASS;
  ros_msg->status.service |= sm::NavSatStatus::SERVICE_GALILEO;
  ros_msg->position_covariance_type = sm::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  Eigen::Vector3d x_ecef, x_wgs84;
  lrm::convertCartesianPoint<msg_base_pos_ecef_t>(sbp_msg, &x_ecef);

  if (!geotf_handler_.get()) return false;
  geotf_handler_->getGeoTf().convertEcefToWgs84(x_ecef, &x_wgs84);

  ros_msg->latitude = x_wgs84.x();
  ros_msg->longitude = x_wgs84.y();
  ros_msg->altitude = x_wgs84.z();

  return true;
}

}  // namespace piksi_multi_cpp
