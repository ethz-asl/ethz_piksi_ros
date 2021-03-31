#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_BASE_POS_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_BASE_POS_RELAY_H_

#include <geometry_msgs/PointStamped.h>
#include <libsbp/navigation.h>
#include <sensor_msgs/NavSatFix.h>
#include "piksi_multi_cpp/sbp_callback_handler/geotf_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class RosBasePosEcefRelay
    : public SBPCallbackHandlerRelay<msg_base_pos_ecef_t,
                                     geometry_msgs::PointStamped> {
 public:
  RosBasePosEcefRelay(const ros::NodeHandle& nh,
                      const std::shared_ptr<sbp_state_t>& state);

 private:
  bool convertSbpToRos(const msg_base_pos_ecef_t& sbp_msg, const uint8_t len,
                       geometry_msgs::PointStamped* ros_msg) override;
};

class RosBasePosLlhRelay
    : public SBPCallbackHandlerRelay<msg_base_pos_ecef_t,
                                     sensor_msgs::NavSatFix> {
 public:
  RosBasePosLlhRelay(const ros::NodeHandle& nh,
                     const std::shared_ptr<sbp_state_t>& state,
                     const GeoTfHandler::Ptr& geotf_handler);

 private:
  bool convertSbpToRos(const msg_base_pos_ecef_t& sbp_msg, const uint8_t len,
                       sensor_msgs::NavSatFix* ros_msg) override;

  GeoTfHandler::Ptr geotf_handler_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_BASE_POS_RELAY_H_
