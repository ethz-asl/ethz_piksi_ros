#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_mag_relay.h"

#include <ros/assert.h>

namespace piksi_multi_cpp {

RosMagRelay::RosMagRelay(const ros::NodeHandle& nh,
                         const std::shared_ptr<sbp_state_t>& state,
                         const RosTimeHandler::Ptr& ros_time_handler)
    : SBPCallbackHandlerRelay<msg_mag_raw_t, sensor_msgs::MagneticField>(
          nh, SBP_MSG_MAG_RAW, state, "ros/mag"),
      ros_time_handler_(ros_time_handler) {}

bool RosMagRelay::convertSbpToRos(const msg_mag_raw_t& sbp_msg,
                                  const uint8_t len,
                                  sensor_msgs::MagneticField* mag) {
  ROS_ASSERT(mag);

  // Invalidate.
  mag->magnetic_field_covariance[0] = -1.0;

  if (!ros_time_handler_.get()) {
    ROS_ERROR("No time handler set.");
    return false;
  }

  // Field conversion.
  mag->header.stamp = ros_time_handler_->lookupTime(sbp_msg.tow, sbp_msg.tow_f);
  mag->header.frame_id = nh_.getUnresolvedNamespace() + "_mag";

  const double kSensorSensitivity = 1.0 / std::numeric_limits<int16_t>::max();
  const double kFromMicro = 1.0e-6;
  const double kMagScaleXY = 1300.0 * kFromMicro * kSensorSensitivity;
  const double kMagScaleZ = 2500.0 * kFromMicro * kSensorSensitivity;

  mag->magnetic_field.x = sbp_msg.mag_x * kMagScaleXY;
  mag->magnetic_field.y = sbp_msg.mag_y * kMagScaleXY;
  mag->magnetic_field.z = sbp_msg.mag_z * kMagScaleZ;
  mag->magnetic_field_covariance[0] = 0.0;  // Validate.

  return true;
}

}  // namespace piksi_multi_cpp
