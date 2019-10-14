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
