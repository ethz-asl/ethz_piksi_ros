#include <eigen_conversions/eigen_msg.h>
#include <libsbp_ros_msgs/ros_conversion.h>
#include <ros/assert.h>
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

  updateEnuOriginFromEcef(x_ecef);
  convertPositionEcefToEnu(x_ecef, &x_enu);

  tf::pointEigenToMsg(x_enu, out->point);
}

}  // namespace piksi_multi_cpp
