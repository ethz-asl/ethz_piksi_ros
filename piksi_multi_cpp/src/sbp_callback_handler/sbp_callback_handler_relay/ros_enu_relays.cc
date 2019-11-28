#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_enu_relays.h"

namespace piksi_multi_cpp {

bool RosPosEnuRelay::convertSbpMsgToRosMsg(const msg_pos_ecef_t& in,
                                           const uint8_t len,
                                           geometry_msgs::PointStamped* out) {
  ROS_ASSERT(out);

  return convertEcefToEnu(in, &out->point);
}

}  // namespace piksi_multi_cpp
