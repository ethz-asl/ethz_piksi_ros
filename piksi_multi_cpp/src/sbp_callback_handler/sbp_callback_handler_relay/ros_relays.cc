#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_relays.h"

#include <libsbp_ros_msgs/ros_conversion.h>

namespace piksi_multi_cpp {

namespace lrm = libsbp_ros_msgs;
namespace gm = geometry_msgs;

void RosPosEcefRelay::convertSbpMsgToRosMsg(const msg_pos_ecef_t& in,
                                            const uint8_t len,
                                            geometry_msgs::PointStamped* out) {
  lrm::convertCartesianPoint<msg_pos_ecef_t, gm::Point>(in, &(out->point));
}

}  // namespace piksi_multi_cpp
