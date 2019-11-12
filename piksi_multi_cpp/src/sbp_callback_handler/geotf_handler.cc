#include "piksi_multi_cpp/sbp_callback_handler/geotf_handler.h"

#include <functional>

#include <libsbp_ros_msgs/ros_conversion.h>

namespace piksi_multi_cpp {
namespace s = std::placeholders;
namespace lrm = libsbp_ros_msgs;

GeoTfHandler::GeoTfHandler(const std::shared_ptr<sbp_state_t>& state)
    : pos_llh_handler_{
          std::bind(&GeoTfHandler::callbackToPosLlh, this, s::_1, s::_2),
          SBP_MSG_POS_LLH, state} {
  geotf_.addFrameByEPSG("ecef", 4978);
  geotf_.addFrameByEPSG("wgs84", 4326);

  ros::NodeHandle nh_node("~");
  nh_node.param<bool>("use_base_enu_origin", use_base_enu_origin_,
                      use_base_enu_origin_);
}

void GeoTfHandler::callbackToPosLlh(const msg_pos_llh_t& msg,
                                    const uint8_t len) {
  if (use_base_enu_origin_) return;  // Wait for base station position instead.
  if (geotf_.hasFrame("enu")) return;         // Already set.
  if (((msg.flags >> 0) & 0x7) == 0) return;  // No nav solution.

  Eigen::Vector3d x_wgs84;
  lrm::convertWgs84Point<msg_pos_llh_t>(msg, &x_wgs84);
  geotf_.addFrameByENUOrigin("enu", x_wgs84.x(), x_wgs84.y(), x_wgs84.z());
}

}  // namespace piksi_multi_cpp
