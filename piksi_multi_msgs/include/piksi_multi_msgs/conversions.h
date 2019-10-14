#ifndef PIKSI_MULTI_MSGS_CONVERSIONS_H_
#define PIKSI_MULTI_MSGS_CONVERSIONS_H_

#include <piksi_multi_msgs/GpsTow.h>
#include <piksi_multi_msgs/Vector3Int.h>

namespace piksi_multi_msgs {
piksi_multi_msgs::Vector3Int
convertSbpVector3IntToRos(const int16_t x, const int16_t y, const int16_t z);
piksi_multi_msgs::GpsTow convertSbpGpsTowToRos(const uint32_t tow,
                                               const uint8_t tow_f);
} // namespace piksi_multi_msgs

#endif // PIKSI_MULTI_MSGS_CONVERSIONS_H_
