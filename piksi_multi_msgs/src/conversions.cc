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

AccuracyQuaternion piksi_multi_msgs::convertSbpAccuracyQuaternionToRos(
    const float w_accuracy, const float x_accuracy, const float y_accuracy,
    const float z_accuracy) {
  AccuracyQuaternion accuracy_quaternion;
  accuracy_quaternion.w = w_accuracy;
  accuracy_quaternion.x = x_accuracy;
  accuracy_quaternion.y = y_accuracy;
  accuracy_quaternion.z = z_accuracy;
  return accuracy_quaternion;
}

QuaternionInt piksi_multi_msgs::convertSbpOrientationQuatToRos(
    const int32_t w, const int32_t x, const int32_t y, const int32_t z) {
  QuaternionInt q;
  q.w = w;
  q.x = x;
  q.y = y;
  q.z = z;
  return q;
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

Uart piksi_multi_msgs::convertSbpUartToRos(const float tx_throughput,
                                           const float rx_throughput,
                                           const uint16_t crc_error_count,
                                           const uint16_t io_error_count,
                                           const uint8_t tx_buffer_level,
                                           const uint8_t rx_buffer_level) {
  Uart uart;
  uart.tx_throughput = tx_throughput;
  uart.rx_throughput = rx_throughput;
  uart.crc_error_count = crc_error_count;
  uart.io_error_count = io_error_count;
  uart.tx_buffer_level = tx_buffer_level;
  uart.rx_buffer_level = rx_buffer_level;
  return uart;
}

Statistics piksi_multi_msgs::convertSbpStatisticsToRos(const int32_t avg,
                                                       const int32_t min,
                                                       const int32_t max,
                                                       const int32_t current) {
  Statistics stats;
  stats.avg = avg;
  stats.min = min;
  stats.max = max;
  stats.current = current;
  return stats;
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
  for (size_t i = 0; i < sizeof(sbp_msg.obs) / sizeof(*sbp_msg.obs); ++i) {
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

PointWgs84 piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_base_pos_llh_t& sbp_msg) {
  PointWgs84 ros_msg =
      convertSbpPointWgs84ToRos(sbp_msg.lat, sbp_msg.lon, sbp_msg.height);
  return ros_msg;
}

geometry_msgs::Point piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_base_pos_ecef_t& sbp_msg) {
  geometry_msgs::Point ros_msg =
      convertSbpPointToRos(sbp_msg.x, sbp_msg.y, sbp_msg.z);
  return ros_msg;
}

// System
Startup piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_startup_t& sbp_msg) {
  Startup ros_msg;
  ros_msg.cause = sbp_msg.cause;
  ros_msg.startup_type = sbp_msg.startup_type;
  return ros_msg;
}

DgnssStatus piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_dgnss_status_t& sbp_msg) {
  DgnssStatus ros_msg;
  ros_msg.differential_type = (sbp_msg.flags >> 0) & 0xF;
  ros_msg.latency = sbp_msg.latency;
  ros_msg.num_signals = sbp_msg.num_signals;
  ros_msg.source = sbp_msg.source;
  return ros_msg;
}

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

InsStatus piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_ins_status_t& sbp_msg) {
  InsStatus ros_msg;
  ros_msg.mode = (sbp_msg.flags >> 0) & 0x7;
  ros_msg.gnss_fix = (sbp_msg.flags >> 3) & 0x1;
  ros_msg.ins_error_value = (sbp_msg.flags >> 4) & 0xF;
  return ros_msg;
}

// Acquisition
AcqResult piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_acq_result_t& sbp_msg) {
  AcqResult ros_msg;
  ros_msg.cn0 = sbp_msg.cn0;
  ros_msg.cp = sbp_msg.cp;
  ros_msg.cf = sbp_msg.cf;
  ros_msg.sid =
      convertSbpSatelliteIdentifierToRos(sbp_msg.sid.sat, sbp_msg.sid.code);
  return ros_msg;
}

// Linux
CpuState piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_linux_cpu_state_t& sbp_msg) {
  CpuState ros_msg;
  ros_msg.index = sbp_msg.index;
  ros_msg.pid = sbp_msg.pid;
  ros_msg.pcpu = sbp_msg.pcpu;
  ros_msg.tname = sbp_msg.tname;
  ros_msg.cmdline = sbp_msg.cmdline;
  return ros_msg;
}

MemState piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_linux_mem_state_t& sbp_msg) {
  MemState ros_msg;
  ros_msg.index = sbp_msg.index;
  ros_msg.pid = sbp_msg.pid;
  ros_msg.pmem = sbp_msg.pmem;
  ros_msg.tname = sbp_msg.tname;
  ros_msg.cmdline = sbp_msg.cmdline;
  return ros_msg;
}

// Orientation
BaselineHeading piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_baseline_heading_t& sbp_msg) {
  BaselineHeading ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.heading = sbp_msg.heading;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.fix_mode.fix_mode = (sbp_msg.flags >> 0) & 0x7;
  return ros_msg;
}

OrientationQuat piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_orient_quat_t& sbp_msg) {
  OrientationQuat ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.q = convertSbpOrientationQuatToRos(sbp_msg.w, sbp_msg.x, sbp_msg.y,
                                             sbp_msg.z);
  ros_msg.accuracy =
      convertSbpAccuracyQuaternionToRos(sbp_msg.w_accuracy, sbp_msg.x_accuracy,
                                        sbp_msg.y_accuracy, sbp_msg.z_accuracy);
  ros_msg.ins_mode.ins_mode = (sbp_msg.flags >> 0) & 0x7;
  return ros_msg;
}

// Piksi
UartState piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_uart_state_t& sbp_msg) {
  UartState ros_msg;
  ros_msg.uart_a = convertSbpUartToRos(
      sbp_msg.uart_a.tx_throughput, sbp_msg.uart_a.rx_throughput,
      sbp_msg.uart_a.crc_error_count, sbp_msg.uart_a.io_error_count,
      sbp_msg.uart_a.tx_buffer_level, sbp_msg.uart_a.rx_buffer_level);
  ros_msg.uart_b = convertSbpUartToRos(
      sbp_msg.uart_b.tx_throughput, sbp_msg.uart_b.rx_throughput,
      sbp_msg.uart_b.crc_error_count, sbp_msg.uart_b.io_error_count,
      sbp_msg.uart_b.tx_buffer_level, sbp_msg.uart_b.rx_buffer_level);
  ros_msg.uart_b = convertSbpUartToRos(
      sbp_msg.uart_ftdi.tx_throughput, sbp_msg.uart_ftdi.rx_throughput,
      sbp_msg.uart_ftdi.crc_error_count, sbp_msg.uart_ftdi.io_error_count,
      sbp_msg.uart_ftdi.tx_buffer_level, sbp_msg.uart_ftdi.rx_buffer_level);
  ros_msg.latency =
      convertSbpStatisticsToRos(sbp_msg.latency.avg, sbp_msg.latency.lmin,
                                sbp_msg.latency.lmax, sbp_msg.latency.current);
  ros_msg.obs_period = convertSbpStatisticsToRos(
      sbp_msg.obs_period.avg, sbp_msg.obs_period.pmin, sbp_msg.obs_period.pmax,
      sbp_msg.obs_period.current);
  return ros_msg;
}

DeviceMonitor piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_device_monitor_t& sbp_msg) {
  DeviceMonitor ros_msg;
  ros_msg.dev_vin = sbp_msg.dev_vin;
  ros_msg.cpu_vint = sbp_msg.cpu_vint;
  ros_msg.cpu_vaux = sbp_msg.cpu_vaux;
  ros_msg.cpu_temperature = sbp_msg.cpu_temperature;
  ros_msg.fe_temperature = sbp_msg.fe_temperature;
  return ros_msg;
}

// Sbas
SbasRaw piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_sbas_raw_t& sbp_msg) {
  SbasRaw ros_msg;
  ros_msg.sid =
      convertSbpSatelliteIdentifierToRos(sbp_msg.sid.sat, sbp_msg.sid.code);
  ros_msg.tow = sbp_msg.tow;
  ros_msg.message_type = sbp_msg.message_type;
  for (size_t i = 0; i < ros_msg.data.size(); i++) {
    ros_msg.data[i] = sbp_msg.data[i];
  }
  return ros_msg;
}

// Ssr
// Tracking
TrackingStates piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_tracking_state_t& sbp_msg) {
  TrackingStates ros_msg;
  for (size_t i = 0; i < sizeof(sbp_msg.states) / sizeof(*sbp_msg.states);
       ++i) {
    TrackingState state;
    state.sid = convertSbpSatelliteIdentifierToRos(sbp_msg.states[i].sid.sat,
                                                   sbp_msg.states[i].sid.code);
    state.fcn = sbp_msg.states[i].fcn;
    state.cn0 = convertSbpCarrierToNoiseToRos(sbp_msg.states[i].cn0);
    ros_msg.states.push_back(state);
  }
  return ros_msg;
}

MeasurementStates piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_measurement_state_t& sbp_msg) {
  MeasurementStates ros_msg;
  for (size_t i = 0; i < sizeof(sbp_msg.states) / sizeof(*sbp_msg.states);
       ++i) {
    MeasurementState state;
    state.mesid = convertSbpSatelliteIdentifierToRos(
        sbp_msg.states[i].mesid.sat, sbp_msg.states[i].mesid.code);
    state.cn0 = convertSbpCarrierToNoiseToRos(sbp_msg.states[i].cn0);
    ros_msg.states.push_back(state);
  }
  return ros_msg;
}

// User
// Vehicle
