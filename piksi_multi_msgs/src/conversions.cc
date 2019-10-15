#include "piksi_multi_msgs/conversions.h"

#include <cmath>

using namespace piksi_multi_msgs;
// How to shift and mask bits: https://stackoverflow.com/a/27592777

AccuracyTangentPlane piksi_multi_msgs::convertSbpAccuracyTangentPlaneToRos(
    const uint16_t h_accuracy, const uint16_t v_accuracy) {
  AccuracyTangentPlane accuracy_tangent_plane;
  accuracy_tangent_plane.h = h_accuracy;
  accuracy_tangent_plane.v = v_accuracy;
  return accuracy_tangent_plane;
}

AccuracyQuaternion piksi_multi_msgs::convertSbpAccuracyQuaternionPlaneToRos(
    const uint16_t w_accuracy, const uint16_t x_accuracy,
    const uint16_t y_accuracy, const uint16_t z_accuracy) {
  AccuracyQuaternion accuracy_quaternion;
  accuracy_quaternion.w = w_accuracy;
  accuracy_quaternion.x = x_accuracy;
  accuracy_quaternion.y = y_accuracy;
  accuracy_quaternion.z = z_accuracy;
  return accuracy_quaternion;
}

CovCartesian piksi_multi_msgs::convertSbpCovCartesianToRos(
    const float x_x, const float x_y, const float x_z, const float y_y,
    const float y_z, const float z_z) {
  CovCartesian cov_cartesian;
  cov_cartesian.x_x = x_x;
  cov_cartesian.x_y = x_y;
  cov_cartesian.x_z = x_z;
  cov_cartesian.y_y = y_y;
  cov_cartesian.y_z = y_z;
  cov_cartesian.z_z = z_z;
  return cov_cartesian;
}

CovTangentNed piksi_multi_msgs::convertSbpCovTangentNedToRos(
    const float n_n, const float n_e, const float n_d, const float e_e,
    const float e_d, const float d_d) {
  CovTangentNed cov_tangent_ned;
  cov_tangent_ned.n_n = n_n;
  cov_tangent_ned.n_e = n_e;
  cov_tangent_ned.n_d = n_d;
  cov_tangent_ned.e_e = e_e;
  cov_tangent_ned.e_d = e_d;
  cov_tangent_ned.d_d = d_d;
  return cov_tangent_ned;
}

PseudoRange piksi_multi_msgs::convertSbpPseudoRangeToRos(const uint32_t range,
                                                         const bool valid) {
  PseudoRange pseudo_range;
  pseudo_range.range = range;
  pseudo_range.valid = valid;
  return pseudo_range;
}

CarrierPhaseCycles piksi_multi_msgs::convertSbpCarrierPhaseCyclesToRos(
    const int32_t i, const int32_t f, const bool valid,
    const bool half_cycle_ambiguity_resolved) {
  CarrierPhaseCycles cpc;
  cpc.i = i;
  cpc.f = f;
  cpc.valid = valid;
  cpc.half_cycle_ambiguity_resolved = half_cycle_ambiguity_resolved;
  return cpc;
}

Doppler piksi_multi_msgs::convertSbpDopplerToRos(const int32_t i,
                                                 const int32_t f,
                                                 const bool valid) {
  Doppler doppler;
  doppler.i = i;
  doppler.f = f;
  doppler.valid = valid;
  return doppler;
}

CarrierToNoise piksi_multi_msgs::convertSbpCarrierToNoiseToRos(
    const uint8_t density) {
  CarrierToNoise cn;
  cn.density = density;
  return cn;
}

SatelliteIdentifier piksi_multi_msgs::convertSbpSatelliteIdentifierToRos(
    const uint8_t sat, const uint8_t code) {
  SatelliteIdentifier sid;
  sid.sat = sat;
  sid.code = code;
  return sid;
}

GpsTimeValue piksi_multi_msgs::convertSbpGpsTimeValueToRos(
    const uint16_t wn, const uint32_t tow, const int32_t ns_residual) {
  GpsTimeValue gps_time_value;
  gps_time_value.wn = wn;
  gps_time_value.tow = tow;
  gps_time_value.ns_residual = ns_residual;

  return gps_time_value;
}

geometry_msgs::Point piksi_multi_msgs::convertSbpPointToRos(const double x,
                                                            const double y,
                                                            const double z) {
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

PointWgs84 piksi_multi_msgs::convertSbpPointWgs84ToRos(const double lat,
                                                       const double lon,
                                                       const double height) {
  PointWgs84 point_wgs84;
  point_wgs84.lat = lat;
  point_wgs84.lon = lon;
  point_wgs84.height = height;
  return point_wgs84;
}

GpsTow piksi_multi_msgs::convertSbpGpsTowToRos(const uint32_t tow,
                                               const uint8_t tow_f) {
  GpsTow gps_tow;
  gps_tow.tow = tow;
  gps_tow.tow_f = tow_f;
  return gps_tow;
}

Vector3Int piksi_multi_msgs::convertSbpVector3IntToRos(const int16_t x,
                                                       const int16_t y,
                                                       const int16_t z) {
  Vector3Int vector_3_int;
  vector_3_int.x = x;
  vector_3_int.y = y;
  vector_3_int.z = z;
  return vector_3_int;
}

Vector3Int32 piksi_multi_msgs::convertSbpVector3Int32ToRos(const int32_t x,
                                                           const int32_t y,
                                                           const int32_t z) {
  Vector3Int32 vector_3_int_32;
  vector_3_int_32.x = x;
  vector_3_int_32.y = y;
  vector_3_int_32.z = z;
  return vector_3_int_32;
}

VectorNed piksi_multi_msgs::convertSbpVectorNedToRos(const int32_t n,
                                                     const int32_t e,
                                                     const int32_t d) {
  VectorNed vec;
  vec.n = n;
  vec.e = e;
  vec.d = d;
  return vec;
}

// Ext Events.
ExtEvent piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_ext_event_t& sbp_msg) {
  ExtEvent ros_msg;
  ros_msg.time =
      convertSbpGpsTimeValueToRos(sbp_msg.wn, sbp_msg.tow, sbp_msg.ns_residual);
  ros_msg.value = (sbp_msg.flags >> 0) & 0x1;
  ros_msg.quality = (sbp_msg.flags >> 1) & 0x1;
  ros_msg.pin = sbp_msg.pin;
  return ros_msg;
}

// Imu
ImuAux piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_imu_aux_t& sbp_msg) {
  ImuAux ros_msg;
  ros_msg.imu_type = sbp_msg.imu_type;
  // TODO(rikba): Convert temperature to unit.
  ros_msg.temp = sbp_msg.temp;
  // Convert accelerometer range (lower four bits).
  uint8_t acc_conf = (sbp_msg.imu_conf >> 0) & 0xF;
  ros_msg.accelerometer_range = std::pow(2, (acc_conf + 1));
  // Convert gyro range (upper four bits).
  uint8_t gyro_conf = (sbp_msg.imu_conf >> 4) & 0xF;
  ros_msg.gyroscope_range = 2000 / std::pow(2, (gyro_conf));

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
  ros_msg.level = sbp_msg.level;
  ros_msg.text = sbp_msg.text;
  return ros_msg;
}

Fwd piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_fwd_t& sbp_msg) {
  Fwd ros_msg;
  ros_msg.source = sbp_msg.source;
  ros_msg.protocol = sbp_msg.protocol;
  ros_msg.fwd_payload = sbp_msg.fwd_payload;
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
  ros_msg.time_source.source = (sbp_msg.flags >> 0) & 0x7;
  return ros_msg;
}

UtcTime piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_utc_time_t& sbp_msg) {
  UtcTime ros_msg;
  ros_msg.time_source.source = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.offset_source = (sbp_msg.flags >> 0) & 0x3;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.year = sbp_msg.year;
  ros_msg.month = sbp_msg.month;
  ros_msg.hours = sbp_msg.hours;
  ros_msg.minutes = sbp_msg.minutes;
  ros_msg.seconds = sbp_msg.seconds;
  ros_msg.ns = sbp_msg.ns;
  return ros_msg;
}

Dops piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_dops_t& sbp_msg) {
  Dops ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.gdop = sbp_msg.gdop;
  ros_msg.pdop = sbp_msg.pdop;
  ros_msg.tdop = sbp_msg.tdop;
  ros_msg.hdop = sbp_msg.hdop;
  ros_msg.vdop = sbp_msg.vdop;
  ros_msg.fix_mode.fix_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.raim_exclusion = (sbp_msg.flags >> 7) & 0x1;
  return ros_msg;
}

PosEcef piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_pos_ecef_t& sbp_msg) {
  PosEcef ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.coordinate_ecef =
      convertSbpPointToRos(sbp_msg.x, sbp_msg.y, sbp_msg.z);
  ros_msg.accuracy = sbp_msg.accuracy;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

PosEcefCov piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_pos_ecef_cov_t& sbp_msg) {
  PosEcefCov ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.coordinate_ecef =
      convertSbpPointToRos(sbp_msg.x, sbp_msg.y, sbp_msg.z);
  ros_msg.cov = convertSbpCovCartesianToRos(sbp_msg.cov_x_x, sbp_msg.cov_x_y,
                                            sbp_msg.cov_x_z, sbp_msg.cov_y_y,
                                            sbp_msg.cov_y_z, sbp_msg.cov_z_z);
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

PosLlh piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_pos_llh_t& sbp_msg) {
  PosLlh ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.coordinate_wgs84 =
      convertSbpPointWgs84ToRos(sbp_msg.lat, sbp_msg.lon, sbp_msg.height);
  ros_msg.accuracy = convertSbpAccuracyTangentPlaneToRos(sbp_msg.h_accuracy,
                                                         sbp_msg.v_accuracy);
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

PosLlhCov piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_pos_llh_cov_t& sbp_msg) {
  PosLlhCov ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.coordinate_wgs84 =
      convertSbpPointWgs84ToRos(sbp_msg.lat, sbp_msg.lon, sbp_msg.height);
  ros_msg.cov = convertSbpCovTangentNedToRos(sbp_msg.cov_n_n, sbp_msg.cov_n_e,
                                             sbp_msg.cov_n_d, sbp_msg.cov_e_e,
                                             sbp_msg.cov_e_d, sbp_msg.cov_d_d);
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

BaselineEcef piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_baseline_ecef_t& sbp_msg) {
  BaselineEcef ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.baseline_ecef =
      convertSbpVector3Int32ToRos(sbp_msg.x, sbp_msg.y, sbp_msg.z);
  ros_msg.accuracy = sbp_msg.accuracy;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode = (sbp_msg.flags >> 0) & 0x7;
  return ros_msg;
}

BaselineNed piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_baseline_ned_t& sbp_msg) {
  BaselineNed ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.baseline_ned =
      convertSbpVectorNedToRos(sbp_msg.n, sbp_msg.e, sbp_msg.d);
  ros_msg.accuracy = convertSbpAccuracyTangentPlaneToRos(sbp_msg.h_accuracy,
                                                         sbp_msg.v_accuracy);
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode = (sbp_msg.flags >> 0) & 0x7;
  return ros_msg;
}

VelEcef piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_vel_ecef_t& sbp_msg) {
  VelEcef ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.velocity_ecef =
      convertSbpVector3Int32ToRos(sbp_msg.x, sbp_msg.y, sbp_msg.z);
  ros_msg.accuracy = sbp_msg.accuracy;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.velocity_mode.velocity_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

VelEcefCov piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_vel_ecef_cov_t& sbp_msg) {
  VelEcefCov ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.velocity_ecef =
      convertSbpVector3Int32ToRos(sbp_msg.x, sbp_msg.y, sbp_msg.z);
  ros_msg.cov = convertSbpCovCartesianToRos(sbp_msg.cov_x_x, sbp_msg.cov_x_y,
                                            sbp_msg.cov_x_z, sbp_msg.cov_y_y,
                                            sbp_msg.cov_y_z, sbp_msg.cov_z_z);
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.velocity_mode.velocity_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

VelNed piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_vel_ned_t& sbp_msg) {
  VelNed ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.velocity_ned =
      convertSbpVectorNedToRos(sbp_msg.n, sbp_msg.e, sbp_msg.d);
  ros_msg.accuracy = convertSbpAccuracyTangentPlaneToRos(sbp_msg.h_accuracy,
                                                         sbp_msg.v_accuracy);
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.velocity_mode.velocity_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

VelNedCov piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_vel_ned_cov_t& sbp_msg) {
  VelNedCov ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.velocity_ned =
      convertSbpVectorNedToRos(sbp_msg.n, sbp_msg.e, sbp_msg.d);
  ros_msg.cov = convertSbpCovTangentNedToRos(sbp_msg.cov_n_n, sbp_msg.cov_n_e,
                                             sbp_msg.cov_n_d, sbp_msg.cov_e_e,
                                             sbp_msg.cov_e_d, sbp_msg.cov_d_d);
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.velocity_mode.velocity_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

VelBody piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_vel_body_t& sbp_msg) {
  VelBody ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.velocity_body =
      convertSbpVector3Int32ToRos(sbp_msg.x, sbp_msg.y, sbp_msg.z);
  ros_msg.cov = convertSbpCovCartesianToRos(sbp_msg.cov_x_x, sbp_msg.cov_x_y,
                                            sbp_msg.cov_x_z, sbp_msg.cov_y_y,
                                            sbp_msg.cov_y_z, sbp_msg.cov_z_z);
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.velocity_mode.velocity_mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 3) & 0x3;
  return ros_msg;
}

AgeCorrections piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_age_corrections_t& sbp_msg) {
  AgeCorrections ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.age = sbp_msg.age;
  return ros_msg;
}

// Observation
Obs piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_obs_t& sbp_msg) {
  Obs ros_msg;
  ros_msg.time = convertSbpGpsTimeValueToRos(
      sbp_msg.header.t.wn, sbp_msg.header.t.tow, sbp_msg.header.t.ns_residual);
  ros_msg.n_obs = sbp_msg.header.n_obs;
  for (size_t i = 0; i < ros_msg.n_obs; ++i) {
    GpsObservation gps_obs;
    gps_obs.P = convertSbpPseudoRangeToRos(sbp_msg.obs[i].P,
                                           (sbp_msg.obs[i].flags >> 0) & 0x1);
    gps_obs.L = convertSbpCarrierPhaseCyclesToRos(
        sbp_msg.obs[i].L.i, sbp_msg.obs[i].L.f,
        (sbp_msg.obs[i].flags >> 1) & 0x1, (sbp_msg.obs[i].flags >> 2) & 0x1);
    gps_obs.D = convertSbpDopplerToRos(sbp_msg.obs[i].D.i, sbp_msg.obs[i].D.f,
                                       (sbp_msg.obs[i].flags >> 3) & 0x1);
    gps_obs.cn0 = convertSbpCarrierToNoiseToRos(sbp_msg.obs[i].cn0);
    gps_obs.lock = sbp_msg.obs[i].lock;
    gps_obs.raim_exclusion = (sbp_msg.obs[i].flags >> 7) & 0x1;
    gps_obs.sid = convertSbpSatelliteIdentifierToRos(sbp_msg.obs[i].sid.sat,
                                                     sbp_msg.obs[i].sid.code);
    ros_msg.obs.push_back(gps_obs);
  }

  return ros_msg;
}

// System
Heartbeat piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_heartbeat_t& sbp_msg) {
  Heartbeat ros_msg;

  ros_msg.system_error = (sbp_msg.flags >> 0) & 0x1;
  ros_msg.io_error = (sbp_msg.flags >> 1) & 0x1;
  ros_msg.swift_nap_error = (sbp_msg.flags >> 2) & 0x1;
  ros_msg.sbp_minor_version = (sbp_msg.flags >> 8) & 0xFF;
  ros_msg.sbp_major_version = (sbp_msg.flags >> 16) & 0xFF;
  ros_msg.short_detected = (sbp_msg.flags >> 30) & 0x1;
  ros_msg.external_antenna_present = (sbp_msg.flags >> 31) & 0x1;

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
