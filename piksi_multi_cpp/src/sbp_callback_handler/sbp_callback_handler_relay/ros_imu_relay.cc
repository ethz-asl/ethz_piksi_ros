#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_imu_relay.h"

namespace piksi_multi_cpp {

RosImuRelay::RosImuRelay(const ros::NodeHandle& nh,
                         const std::shared_ptr<sbp_state_t>& state,
                         const RosTimeHandler::Ptr& ros_time_handler)
    : SBPCallbackHandlerRelay<msg_imu_raw_t, sensor_msgs::Imu>(
          nh, SBP_MSG_IMU_RAW, state, "ros/imu"),
      ros_time_handler_(ros_time_handler),
      imu_aux_handler_{std::bind(&RosImuRelay::callbackToImuAux, this,
                                 std::placeholders::_1, std::placeholders::_2),
                       SBP_MSG_IMU_AUX, state} {}

bool RosImuRelay::convertSbpToRos(const msg_imu_raw_t& sbp_msg,
                                  const uint8_t len, sensor_msgs::Imu* imu) {
  ROS_ASSERT(imu);

  // Invalidate.
  imu->orientation_covariance[0] = -1.0;
  imu->angular_velocity_covariance[0] = -1.0;
  imu->linear_acceleration_covariance[0] = -1.0;

  if (!ros_time_handler_.get()) {
    ROS_ERROR("No time handler set.");
    return false;
  }

  if (!gyro_scale_.has_value() || !acc_scale_.has_value()) {
    ROS_DEBUG("Did not receive IMU configuration data.");
    return false;
  }

  // Field conversion.
  imu->header.stamp = ros_time_handler_->lookupTime(sbp_msg.tow, sbp_msg.tow_f);
  imu->header.frame_id = nh_.getUnresolvedNamespace() + "_imu";
  imu->angular_velocity.x = sbp_msg.gyr_x * gyro_scale_.value();
  imu->angular_velocity.y = sbp_msg.gyr_y * gyro_scale_.value();
  imu->angular_velocity.z = sbp_msg.gyr_z * gyro_scale_.value();
  imu->angular_velocity_covariance[0] = 0.0;  // Validate.

  imu->linear_acceleration.x = sbp_msg.acc_x * acc_scale_.value();
  imu->linear_acceleration.y = sbp_msg.acc_y * acc_scale_.value();
  imu->linear_acceleration.z = sbp_msg.acc_z * acc_scale_.value();
  imu->linear_acceleration_covariance[0] = 0.0;  // Validate.

  return true;
}

void RosImuRelay::callbackToImuAux(const msg_imu_aux_t& msg,
                                   const uint8_t len) {
  // Update configurations once at startup and then only if someone is
  // subscribing.
  if ((acc_scale_.has_value() && gyro_scale_.has_value()) &&
      (!relay_pub_.has_value() || relay_pub_.value().getNumSubscribers() == 0))
    return;

  const double kSensorSensitivity = 1.0 / std::numeric_limits<int16_t>::max();
  const double kGravity = 9.81;
  const double kDegToRad = M_PI / 180.0;
  const double kAccPrescale = kGravity * kSensorSensitivity;
  const double kGyroPrescale = kDegToRad * kSensorSensitivity;

  uint8_t acc_conf = ((msg.imu_conf >> 0) & 0xF) + 1;
  // 2^acc_conf == 1 << acc_conf
  acc_scale_ = std::make_optional((1 << acc_conf) * kAccPrescale);

  uint8_t gyro_conf = ((msg.imu_conf >> 4) & 0xF);
  gyro_scale_ =
      std::make_optional(2000.0 / (gyro_conf * gyro_conf) * kGyroPrescale);
}

}  // namespace piksi_multi_cpp
