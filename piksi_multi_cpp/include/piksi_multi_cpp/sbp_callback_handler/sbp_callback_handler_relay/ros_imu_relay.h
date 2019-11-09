#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_IMU_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_IMU_RELAY_H_

#include <libsbp/imu.h>
#include <sensor_msgs/Imu.h>
#include "piksi_multi_cpp/sbp_callback_handler/ros_time_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class RosImuRelay
    : public SBPCallbackHandlerRelay<msg_imu_raw_t, sensor_msgs::Imu> {
 public:
  RosImuRelay(const ros::NodeHandle& nh,
              const std::shared_ptr<sbp_state_t>& state,
              const RosTimeHandler::Ptr& ros_time_handler);

 private:
  bool convertSbpToRos(const msg_imu_raw_t& sbp_msg, const uint8_t len,
                       sensor_msgs::Imu* ros_msg) override;

  void callbackToImuAux(const msg_imu_aux_t& msg, const uint8_t len);

  RosTimeHandler::Ptr ros_time_handler_;
  SBPLambdaCallbackHandler<msg_imu_aux_t> imu_aux_handler_;

  std::optional<double> acc_scale_;
  std::optional<double> gyro_scale_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_IMU_RELAY_H_
