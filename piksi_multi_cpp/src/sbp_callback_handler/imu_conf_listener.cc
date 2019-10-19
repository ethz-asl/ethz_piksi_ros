#include "piksi_multi_cpp/sbp_callback_handler/imu_conf_listener.h"

#include <libsbp/navigation.h>
#include <libsbp_ros_msgs/ros_conversion.h>
#include <ros/assert.h>
#include <limits>

namespace piksi_multi_cpp {

const double kSensorSensitivity = 1.0 / std::numeric_limits<int16_t>::max();
const double kGravity = 9.81;
const double kDegToRad = M_PI / 180.0;
const double kFromMicro = 1.0e-6;
const double kFromMilli = 1.0e-3;

const double kAccPrescale = kGravity * kSensorSensitivity;
const double kGyroPrescale = kDegToRad * kSensorSensitivity;
const double kTempPrescale = 5.0 / 9.0 * 1.0e-2;
const double kMagScaleXY = 1300.0 * kFromMicro * kSensorSensitivity;
const double kMagScaleZ = 2500.0 * kFromMicro * kSensorSensitivity;

ImuConfListener::ImuConfListener(const ros::NodeHandle& nh,
                                 const std::shared_ptr<sbp_state_t>& state)
    : SBPCallbackHandler(nh, SBP_MSG_IMU_AUX, state) {}

bool ImuConfListener::getAccelerometerScale(double* acc_scale) const {
  ROS_ASSERT(acc_scale);
  if (!imu_aux_.has_value()) return false;
  uint8_t acc_conf = ((imu_aux_.value().imu_conf >> 0) & 0xF) + 1;
  uint8_t acc_range = acc_conf * acc_conf;
  *acc_scale = acc_range * kAccPrescale;
  return true;
}

bool ImuConfListener::getGyroscopeScale(double* gyro_scale) const {
  ROS_ASSERT(gyro_scale);
  if (!imu_aux_.has_value()) return false;
  uint8_t gyro_conf = ((imu_aux_.value().imu_conf >> 4) & 0xF);
  uint16_t gyro_range = 2000 / (gyro_conf * gyro_conf);
  *gyro_scale = gyro_range * kGyroPrescale;
  return true;
}

bool ImuConfListener::getImuTemp(double* temp) const {
  ROS_ASSERT(temp);
  if (!imu_aux_.has_value()) return false;
  // TODO(rikba) Check whether the temperature is actually given in Fahrenheit.
  *temp = (imu_aux_.value().temp - 32) * kTempPrescale;
  return true;
}

bool ImuConfListener::getImuType(uint8_t* type) const {
  ROS_ASSERT(type);
  if (!imu_aux_.has_value()) return false;
  *type = imu_aux_.value().imu_type;
  return true;
}

void ImuConfListener::callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) {
  // Cast message.
  auto sbp_msg = (msg_imu_aux_t*)msg;
  if (!sbp_msg) {
    ROS_WARN("Cannot cast SBP message.");
    return;
  }
  imu_aux_.value() = *sbp_msg;
}

}  // namespace piksi_multi_cpp
