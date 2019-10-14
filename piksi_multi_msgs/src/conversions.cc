#include "piksi_multi_msgs/conversions.h"

#include <cmath>

using namespace piksi_multi_msgs;
// How to shift and mask bits: https://stackoverflow.com/a/27592777

const double kFromMilli = 0.001;

AccuracyTangentPlane piksi_multi_msgs::convertSbpAccuracyTangentPlaneToRos(
    const uint16_t h_accuracy, const uint16_t v_accuracy) {
  AccuracyTangentPlane accuracy_tangent_plane;
  accuracy_tangent_plane.h.data = kFromMilli * h_accuracy;
  accuracy_tangent_plane.v.data = kFromMilli * v_accuracy;
  return accuracy_tangent_plane;
}

AccuracyQuaternion piksi_multi_msgs::convertSbpAccuracyQuaternionPlaneToRos(
    const uint16_t w_accuracy, const uint16_t x_accuracy,
    const uint16_t y_accuracy, const uint16_t z_accuracy) {
  AccuracyQuaternion accuracy_quaternion;
  accuracy_quaternion.w.data = w_accuracy;
  accuracy_quaternion.x.data = x_accuracy;
  accuracy_quaternion.y.data = y_accuracy;
  accuracy_quaternion.z.data = z_accuracy;
  return accuracy_quaternion;
}

CovCartesian piksi_multi_msgs::convertSbpCovCartesianToRos(
    const float x_x, const float x_y, const float x_z, const float y_y,
    const float y_z, const float z_z) {
  CovCartesian cov_cartesian;
  cov_cartesian.x_x.data = x_x;
  cov_cartesian.x_y.data = x_y;
  cov_cartesian.x_z.data = x_z;
  cov_cartesian.y_y.data = y_y;
  cov_cartesian.y_z.data = y_z;
  cov_cartesian.z_z.data = z_z;
  return cov_cartesian;
}

CovTangentNed piksi_multi_msgs::convertSbpCovTangentNedToRos(
    const float n_n, const float n_e, const float n_d, const float e_e,
    const float e_d, const float d_d) {
  CovTangentNed cov_tangent_ned;
  cov_tangent_ned.n_n.data = n_n;
  cov_tangent_ned.n_e.data = n_e;
  cov_tangent_ned.n_d.data = n_d;
  cov_tangent_ned.e_e.data = e_e;
  cov_tangent_ned.e_d.data = e_d;
  cov_tangent_ned.d_d.data = d_d;
  return cov_tangent_ned;
}

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

PointEcef piksi_multi_msgs::convertSbpPointEcefToRos(const double x,
                                                     const double y,
                                                     const double z) {
  PointEcef point_ecef;
  point_ecef.x.data = x;
  point_ecef.y.data = y;
  point_ecef.z.data = z;
  return point_ecef;
}

PointNed piksi_multi_msgs::convertSbpPointNedToRos(const int32_t n,
                                                   const int32_t e,
                                                   const int32_t d) {
  PointNed point_ned;
  point_ned.n.data = n * kFromMilli;
  point_ned.e.data = e * kFromMilli;
  point_ned.d.data = d * kFromMilli;
  return point_ned;
}

PointWgs84 piksi_multi_msgs::convertSbpPointWgs84ToRos(const double lat,
                                                       const double lon,
                                                       const double height) {
  PointWgs84 point_wgs84;
  point_wgs84.lat.data = lat;
  point_wgs84.lon.data = lon;
  point_wgs84.height.data = height;
  return point_wgs84;
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

// Ext Events.
ExtEvent piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_ext_event_t& sbp_msg) {
  ExtEvent ros_msg;
  ros_msg.time =
      convertSbpGpsTimeValueToRos(sbp_msg.wn, sbp_msg.tow, sbp_msg.ns_residual);
  ros_msg.quality.data = (sbp_msg.flags >> 1) & 0x1;
  ros_msg.pin = convertSbpGpioToRos(sbp_msg.pin, (sbp_msg.flags >> 0) & 0x1);
  return ros_msg;
}

// Imu
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

// Logging
Log piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_log_t& sbp_msg) {
  Log ros_msg;
  ros_msg.level.data = sbp_msg.level;
  ros_msg.text.data = sbp_msg.text;
  return ros_msg;
}

Fwd piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_fwd_t& sbp_msg) {
  Fwd ros_msg;
  ros_msg.source.data = sbp_msg.source;
  ros_msg.protocol.data = sbp_msg.protocol;
  ros_msg.fwd_payload.data = sbp_msg.fwd_payload;
  return ros_msg;
}

// Mag
MagRaw piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_mag_raw_t& sbp_msg) {
  MagRaw ros_msg;
  ros_msg.tow = convertSbpGpsTowToRos(sbp_msg.tow, sbp_msg.tow_f);
  ros_msg.mag =
      convertSbpVector3IntToRos(sbp_msg.mag_x, sbp_msg.mag_y, sbp_msg.mag_z);
  return ros_msg;
}

// Navigation
GpsTime piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_gps_time_t& sbp_msg) {
  GpsTime ros_msg;
  ros_msg.time =
      convertSbpGpsTimeValueToRos(sbp_msg.wn, sbp_msg.tow, sbp_msg.ns_residual);
  ros_msg.source.source.data = (sbp_msg.flags >> 0) & 0x7;
  return ros_msg;
}

UtcTime piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_utc_time_t& sbp_msg) {
  UtcTime ros_msg;
  ros_msg.time_source.source.data = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.offset_source.data = (sbp_msg.flags >> 0) & 0x3;
  ros_msg.tow.data = sbp_msg.tow;
  ros_msg.year.data = sbp_msg.year;
  ros_msg.month.data = sbp_msg.month;
  ros_msg.hours.data = sbp_msg.hours;
  ros_msg.minutes.data = sbp_msg.minutes;
  ros_msg.seconds.data = sbp_msg.seconds;
  ros_msg.ns.data = sbp_msg.ns;
  return ros_msg;
}

Dops piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_dops_t& sbp_msg) {
  Dops ros_msg;
  ros_msg.tow.data = sbp_msg.tow;
  ros_msg.gdop.data = sbp_msg.gdop;
  ros_msg.pdop.data = sbp_msg.pdop;
  ros_msg.tdop.data = sbp_msg.tdop;
  ros_msg.hdop.data = sbp_msg.hdop;
  ros_msg.vdop.data = sbp_msg.vdop;
  ros_msg.fix_mode.fix_mode.data = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.raim_exclusion.data = (sbp_msg.flags >> 7) & 0x1;
  return ros_msg;
}

PosEcef piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_pos_ecef_t& sbp_msg) {
  PosEcef ros_msg;
  ros_msg.tow.data = sbp_msg.tow;
  ros_msg.coordinate =
      convertSbpPointEcefToRos(sbp_msg.x, sbp_msg.y, sbp_msg.z);
  ros_msg.accuracy.data = kFromMilli * sbp_msg.accuracy;
  ros_msg.n_sats.data = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode.data = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode.data = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

PosEcefCov piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_pos_ecef_cov_t& sbp_msg) {
  PosEcefCov ros_msg;
  ros_msg.tow.data = sbp_msg.tow;
  ros_msg.coordinate =
      convertSbpPointEcefToRos(sbp_msg.x, sbp_msg.y, sbp_msg.z);
  ros_msg.cov = convertSbpCovCartesianToRos(sbp_msg.cov_x_x, sbp_msg.cov_x_y,
                                            sbp_msg.cov_x_z, sbp_msg.cov_y_y,
                                            sbp_msg.cov_y_z, sbp_msg.cov_z_z);
  ros_msg.n_sats.data = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode.data = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode.data = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

PosLlh piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_pos_llh_t& sbp_msg) {
  PosLlh ros_msg;
  ros_msg.tow.data = sbp_msg.tow;
  ros_msg.coordinate =
      convertSbpPointWgs84ToRos(sbp_msg.lat, sbp_msg.lon, sbp_msg.height);
  ros_msg.accuracy = convertSbpAccuracyTangentPlaneToRos(sbp_msg.h_accuracy,
                                                         sbp_msg.v_accuracy);
  ros_msg.n_sats.data = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode.data = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode.data = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

PosLlhCov piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_pos_llh_cov_t& sbp_msg) {
  PosLlhCov ros_msg;
  ros_msg.tow.data = sbp_msg.tow;
  ros_msg.coordinate =
      convertSbpPointWgs84ToRos(sbp_msg.lat, sbp_msg.lon, sbp_msg.height);
  ros_msg.cov = convertSbpCovTangentNedToRos(sbp_msg.cov_n_n, sbp_msg.cov_n_e,
                                             sbp_msg.cov_n_d, sbp_msg.cov_e_e,
                                             sbp_msg.cov_e_d, sbp_msg.cov_d_d);
  ros_msg.n_sats.data = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode.data = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode.data = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

BaselineEcef piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_baseline_ecef_t& sbp_msg) {
  BaselineEcef ros_msg;
  ros_msg.tow.data = sbp_msg.tow;
  ros_msg.coordinate = convertSbpPointEcefToRos(
      kFromMilli * sbp_msg.x, kFromMilli * sbp_msg.y, kFromMilli * sbp_msg.z);
  ros_msg.accuracy.data = kFromMilli * sbp_msg.accuracy;
  ros_msg.n_sats.data = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode.data = (sbp_msg.flags >> 0) & 0x7;
  return ros_msg;
}

BaselineNed piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_baseline_ned_t& sbp_msg) {
  BaselineNed ros_msg;
  ros_msg.tow.data = sbp_msg.tow;
  ros_msg.coordinate = convertSbpPointNedToRos(sbp_msg.n, sbp_msg.e, sbp_msg.d);
  ros_msg.accuracy = convertSbpAccuracyTangentPlaneToRos(sbp_msg.h_accuracy,
                                                         sbp_msg.v_accuracy);
  ros_msg.n_sats.data = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode.data = (sbp_msg.flags >> 0) & 0x7;
  return ros_msg;
}

// Observation
// System
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

// Acquisition
// Linux
// Orientation
// Piksi
// Sbas
// Ssr
// Tracking
// User
// Vehicle
