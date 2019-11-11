#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_ENU_RELAYS_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_ENU_RELAYS_H_

#include <libsbp/navigation.h>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_relay.h"

namespace piksi_multi_cpp {

class RosPosEnuRelay
    : public RosRelay<msg_pos_ecef_cov_t,
                      piksi_rtk_msgs::PositionWithCovarianceStamped> {
 public:
  inline RosPosEnuRelay(const ros::NodeHandle& nh,
                        const std::shared_ptr<sbp_state_t>& state,
                        const RosTimeHandler::Ptr& ros_time_handler)
      : RosRelay(nh, SBP_MSG_POS_ECEF, state, "pos_enu", ros_time_handler,
                 "enu") {}

 private:
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_ENU_RELAYS_H_
