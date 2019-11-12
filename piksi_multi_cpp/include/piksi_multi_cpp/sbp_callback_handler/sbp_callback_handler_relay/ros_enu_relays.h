#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_ENU_RELAYS_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_ENU_RELAYS_H_

#include <geometry_msgs/PointStamped.h>
#include <geotf/geodetic_converter.h>
#include <libsbp/navigation.h>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <optional>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_relay.h"

namespace piksi_multi_cpp {

// Per default the ENU origin will be the base station position. But it can also
// be set to the current rover position or a user defined position.
template <class SbpMsgType, class RosMsgType>
class RosEnuRelay : public RosRelay<SbpMsgType, RosMsgType> {
 public:
  inline RosEnuRelay(const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
                     const std::shared_ptr<sbp_state_t>& state,
                     const std::string& topic,
                     const RosTimeHandler::Ptr& ros_time_handler)
      : RosRelay<SbpMsgType, RosMsgType>(nh, sbp_msg_type, state, topic,
                                         ros_time_handler, "enu") {
    geotf_.addFrameByEPSG("ecef", 4978);
    geotf_.addFrameByEPSG("wgs84", 4326);
    ros::NodeHandle nh_node("~");
    nh_node.param<bool>("use_base_enu_origin", use_base_enu_origin_,
                        use_base_enu_origin_);
  }

 public:
  inline void setEnuOriginWgs84(const double lat, const double lon,
                                const double alt) {
    geotf_.addFrameByENUOrigin("enu", lat, lon, alt);
  }

  inline void resetEnuOrigin() { geotf_.removeFrame("enu"); }

 protected:
  bool use_base_enu_origin_ = true;  // False: ENU origin is first position.
  geotf::GeodeticConverter geotf_;

 private:
};

class RosPosEnuRelay
    : public RosEnuRelay<msg_pos_ecef_t, geometry_msgs::PointStamped> {
 public:
  inline RosPosEnuRelay(const ros::NodeHandle& nh,
                        const std::shared_ptr<sbp_state_t>& state,
                        const RosTimeHandler::Ptr& ros_time_handler)
      : RosEnuRelay(nh, SBP_MSG_POS_ECEF, state, "pos_enu", ros_time_handler) {}

 private:
  void convertSbpMsgToRosMsg(const msg_pos_ecef_t& in, const uint8_t len,
                             geometry_msgs::PointStamped* out) override;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_ENU_RELAYS_H_
