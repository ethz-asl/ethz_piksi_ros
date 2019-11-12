#include <eigen_conversions/eigen_msg.h>
#include <libsbp_ros_msgs/ros_conversion.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_enu_relays.h"

namespace piksi_multi_cpp {

namespace lrm = libsbp_ros_msgs;

void RosPosEnuRelay::convertSbpMsgToRosMsg(const msg_pos_ecef_t& in,
                                           const uint8_t len,
                                           geometry_msgs::PointStamped* out) {
  ROS_ASSERT(out);

  // Convert measurement to Eigen.
  Eigen::Vector3d x_ecef, x_enu;
  lrm::convertCartesianPoint<msg_pos_ecef_t>(in, &x_ecef);

  // Update ENU origin if it is not to be set from base station position.
  if (!use_base_enu_origin_ && !geotf_.hasFrame("enu")) {
    Eigen::Vector3d x_wgs84;
    if (!geotf_.convert("ecef", x_ecef, "wgs84", &x_wgs84)) {
      ROS_ERROR("Failed to convert ECEF to WGS84.");
      return;
    }
    geotf_.addFrameByENUOrigin("enu", x_wgs84.x(), x_wgs84.y(), x_wgs84.z());
  }

  if (!geotf_.canConvert("ecef", "enu")) {
    ROS_WARN_THROTTLE(5.0,
                      "Cannot convert ECEF to ENU. Waiting for ENU to be set.");
    return;
  }

  // Convert ECEF to ENU frame.
  if (!geotf_.convert("ecef", x_ecef, "enu", &x_enu)) {
    ROS_ERROR("Failed to convert ECEF to ENU.");
    return;
  }

  // Save result.
  tf::pointEigenToMsg(x_enu, out->point);
}

}  // namespace piksi_multi_cpp
