#include "piksi_multi_msgs/conversions.h"

using namespace piksi_multi_msgs;

Vector3Int piksi_multi_msgs::convertSbpVector3IntToRos(const int16_t x,
                                                       const int16_t y,
                                                       const int16_t z) {
  Vector3Int vector_3_int;
  vector_3_int.x.data = x;
  vector_3_int.y.data = y;
  vector_3_int.z.data = z;

  return vector_3_int;
}

GpsTow piksi_multi_msgs::convertSbpGpsTowToRos(const uint32_t tow,
                                               const uint8_t tow_f) {
  GpsTow gps_tow;
  gps_tow.tow.data = tow;
  gps_tow.tow_f.data = tow_f;
  return gps_tow;
}

Heartbeat piksi_multi_msgs::convertSbpMsgToRosMsg(
    const msg_heartbeat_t& sbp_msg) {
  Heartbeat ros_msg;

  // How to shift and mask bits: https://stackoverflow.com/a/27592777
  ros_msg.system_error.data = (sbp_msg.flags >> 0) & 0x1;
  ros_msg.io_error.data = (sbp_msg.flags >> 1) & 0x1;
  ros_msg.swift_nap_error.data = (sbp_msg.flags >> 2) & 0x1;
  ros_msg.sbp_minor_version.data = (sbp_msg.flags >> 8) & 0xFF;
  ros_msg.sbp_major_version.data = (sbp_msg.flags >> 16) & 0xFF;
  ros_msg.short_detected.data = (sbp_msg.flags >> 30) & 0x1;
  ros_msg.external_antenna_present.data = (sbp_msg.flags >> 31) & 0x1;

  return ros_msg;
}

ImuRaw piksi_multi_msgs::convertSbpMsgToRosMsg(const msg_imu_raw_t& sbp_msg) {
  ImuRaw ros_msg;
  ros_msg.tow =
      piksi_multi_msgs::convertSbpGpsTowToRos(sbp_msg.tow, sbp_msg.tow_f);
  ros_msg.acc = piksi_multi_msgs::convertSbpVector3IntToRos(
      sbp_msg.acc_x, sbp_msg.acc_y, sbp_msg.acc_z);
  ros_msg.gyr = piksi_multi_msgs::convertSbpVector3IntToRos(
      sbp_msg.gyr_x, sbp_msg.gyr_y, sbp_msg.gyr_z);

  return ros_msg;
}
