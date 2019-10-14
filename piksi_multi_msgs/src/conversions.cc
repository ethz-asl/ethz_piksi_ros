#include "piksi_multi_msgs/conversions.h"

#include <cmath>

using namespace piksi_multi_msgs;
// How to shift and mask bits: https://stackoverflow.com/a/27592777

Gpio piksi_multi_msgs::convertSbpGpioToRos(const uint8_t pin,
                                           const bool value) {
  Gpio gpio;
  gpio.number.data = pin;
  gpio.value.data = value;
  return gpio;
}

GpsTimeValue piksi_multi_msgs::convertSbpGpsTimeValueToRos(
    const uint16_t wn, const uint32_t tow, const int32_t ns_residual) {
  GpsTimeValue gps_time_value;
  gps_time_value.wn.data = wn;
  gps_time_value.tow.data = tow;
  gps_time_value.ns_residual.data = ns_residual;

  return gps_time_value;
}

GpsTow piksi_multi_msgs::convertSbpGpsTowToRos(const uint32_t tow,
                                               const uint8_t tow_f) {
  GpsTow gps_tow;
  gps_tow.tow.data = tow;
  gps_tow.tow_f.data = tow_f;
  return gps_tow;
}

Vector3Int piksi_multi_msgs::convertSbpVector3IntToRos(const int16_t x,
                                                       const int16_t y,
                                                       const int16_t z) {
  Vector3Int vector_3_int;
  vector_3_int.x.data = x;
  vector_3_int.y.data = y;
  vector_3_int.z.data = z;

  return vector_3_int;
}

ExtEvent piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_ext_event_t& sbp_msg) {
  ExtEvent ros_msg;
  ros_msg.time =
      convertSbpGpsTimeValueToRos(sbp_msg.wn, sbp_msg.tow, sbp_msg.ns_residual);
  ros_msg.quality.data = (sbp_msg.flags >> 1) & 0x1;
  ros_msg.pin = convertSbpGpioToRos(sbp_msg.pin, (sbp_msg.flags >> 0) & 0x1);
  return ros_msg;
}

ImuAux piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_imu_aux_t& sbp_msg) {
  ImuAux ros_msg;
  ros_msg.imu_type.data = sbp_msg.imu_type;
  // TODO(rikba): Convert temperature to unit.
  ros_msg.temp.data = sbp_msg.temp;
  // Convert accelerometer range (lower four bits).
  uint8_t acc_conf = (sbp_msg.imu_conf >> 0) & 0xF;
  ros_msg.accelerometer_range.data = std::pow(2, (acc_conf + 1));
  // Convert gyro range (upper four bits).
  uint8_t gyro_conf = (sbp_msg.imu_conf >> 4) & 0xF;
  ros_msg.gyroscope_range.data = 2000 / std::pow(2, (gyro_conf));

  return ros_msg;
}

ImuRaw piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_imu_raw_t& sbp_msg) {
  ImuRaw ros_msg;
  ros_msg.tow = convertSbpGpsTowToRos(sbp_msg.tow, sbp_msg.tow_f);
  ros_msg.acc =
      convertSbpVector3IntToRos(sbp_msg.acc_x, sbp_msg.acc_y, sbp_msg.acc_z);
  ros_msg.gyr =
      convertSbpVector3IntToRos(sbp_msg.gyr_x, sbp_msg.gyr_y, sbp_msg.gyr_z);

  return ros_msg;
}

Heartbeat piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_heartbeat_t& sbp_msg) {
  Heartbeat ros_msg;

  ros_msg.system_error.data = (sbp_msg.flags >> 0) & 0x1;
  ros_msg.io_error.data = (sbp_msg.flags >> 1) & 0x1;
  ros_msg.swift_nap_error.data = (sbp_msg.flags >> 2) & 0x1;
  ros_msg.sbp_minor_version.data = (sbp_msg.flags >> 8) & 0xFF;
  ros_msg.sbp_major_version.data = (sbp_msg.flags >> 16) & 0xFF;
  ros_msg.short_detected.data = (sbp_msg.flags >> 30) & 0x1;
  ros_msg.external_antenna_present.data = (sbp_msg.flags >> 31) & 0x1;

  return ros_msg;
}
