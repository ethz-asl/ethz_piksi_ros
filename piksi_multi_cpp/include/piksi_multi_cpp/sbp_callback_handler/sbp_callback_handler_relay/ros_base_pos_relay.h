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
  inline RosBasePosEcefRelay(const ros::NodeHandle& nh,
                             const std::shared_ptr<sbp_state_t>& state);

 private:
  bool convertSbpToRos(const msg_base_pos_ecef_t& sbp_msg, const uint8_t len,
                       geometry_msgs::PointStamped* ros_msg) override;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_BASE_POS_RELAY_H_
