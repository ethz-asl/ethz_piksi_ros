#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_MAG_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_MAG_RELAY_H_

#include <libsbp/mag.h>
#include <sensor_msgs/MagneticField.h>
#include "piksi_multi_cpp/sbp_callback_handler/ros_time_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class RosMagRelay : public SBPCallbackHandlerRelay<msg_mag_raw_t,
                                                   sensor_msgs::MagneticField> {
 public:
  RosMagRelay(const ros::NodeHandle& nh,
              const std::shared_ptr<sbp_state_t>& state,
              const RosTimeHandler::Ptr& ros_time_handler);

 private:
  bool convertSbpToRos(const msg_mag_raw_t& sbp_msg, const uint8_t len,
                       sensor_msgs::MagneticField* ros_msg) override;

  RosTimeHandler::Ptr ros_time_handler_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_MAG_RELAY_H_
