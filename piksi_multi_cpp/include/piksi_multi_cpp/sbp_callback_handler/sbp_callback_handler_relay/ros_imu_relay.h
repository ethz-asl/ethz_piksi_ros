#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_IMU_RELAY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_IMU_RELAY_H_

#include <libsbp/imu.h>
#include <sensor_msgs/Imu.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class RosImuRelay
    : public SBPCallbackHandlerRelay<msg_imu_raw_t, sensor_msgs::Imu> {
 public:
  inline RosImuRelay(const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
                     const std::shared_ptr<sbp_state_t>& state,
                     const std::shared_ptr<UtcTimeBuffer>& utc_time_buffer)
      : SBPCallbackHandlerRelay<msg_imu_raw_t, sensor_msgs::Imu>(
            nh, sbp_msg_type, state, "ros/imu"),
        utc_time_buffer_(utc_time_buffer) {}

 private:
  inline sensor_msgs::Imu convertSbpToRos(const SbpMsgType& sbp_msg,
                                          const uint8_t len) override {
    sensor_msgs::Imu ros_msg;

    // Assign time stamp.
    if (utc_time_buffer_.get())
      ros_msg.header.stamp = utc_time_buffer_->getTime(sbp_msg.tow);
    else {
      ROS_WARN_ONCE("Using ros::Time::now() to stamp navigation data.");
      ros_msg.header.stamp = ros::Time::now();
    }

    // Set frame id.
    ros_msg.header.frame_id = frame_id_;

    // Manual conversion.
    convertSbpMsgToRosMsg(sbp_msg, len, &ros_msg);
    return ros_msg;
  }

  std::shared_ptr<UtcTimeBuffer> utc_time_buffer_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_ROS_IMU_RELAY_H_
