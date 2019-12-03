#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_enu_relays.h"

namespace piksi_multi_cpp {

bool RosPosEnuRelay::convertSbpMsgToRosMsg(const msg_pos_ecef_t& in,
                                           const uint8_t len,
                                           geometry_msgs::PointStamped* out) {
  ROS_ASSERT(out);

  return convertEcefToEnu(in, &out->point);
}

bool RosTransformEnuRelay::convertSbpMsgToRosMsg(
    const msg_pos_ecef_t& in, const uint8_t len,
    geometry_msgs::TransformStamped* out) {
  ROS_ASSERT(out);

  if (!convertEcefToEnu(in, &out->transform.translation)) return false;

  // TODO(rikba): Also add orientation information if available.
  out->transform.rotation.w = 1.0;

  return true;
}

}  // namespace piksi_multi_cpp
