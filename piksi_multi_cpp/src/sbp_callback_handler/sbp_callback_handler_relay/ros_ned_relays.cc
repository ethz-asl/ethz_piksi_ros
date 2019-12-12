#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_ned_relays.h"

namespace piksi_multi_cpp {

bool RosPosNedRelay::convertSbpMsgToRosMsg(const msg_pos_ecef_t& in,
                                           const uint8_t len,
                                           geometry_msgs::PointStamped* out) {
  ROS_ASSERT(out);

  return convertEcefToNed(in, &out->point);
}

bool RosTransformNedRelay::convertSbpMsgToRosMsg(
    const msg_pos_ecef_t& in, const uint8_t len,
    geometry_msgs::TransformStamped* out) {
  ROS_ASSERT(out);

  if (!convertEcefToNed(in, &out->transform.translation)) return false;

  // TODO(rikba): Also add orientation information if available.
  out->transform.rotation.w = 1.0;

  return true;
}

}  // namespace piksi_multi_cpp
