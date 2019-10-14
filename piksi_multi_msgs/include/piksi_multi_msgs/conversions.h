#ifndef PIKSI_MULTI_MSGS_CONVERSIONS_H_
#define PIKSI_MULTI_MSGS_CONVERSIONS_H_

#include "piksi_multi_msgs/GpsTow.h"
#include "piksi_multi_msgs/Heartbeat.h"
#include "piksi_multi_msgs/ImuRaw.h"
#include "piksi_multi_msgs/Vector3Int.h"

#include <libsbp/imu.h>
#include <libsbp/system.h>

namespace piksi_multi_msgs {
Vector3Int convertSbpVector3IntToRos(const int16_t x, const int16_t y,
                                     const int16_t z);
GpsTow convertSbpGpsTowToRos(const uint32_t tow, const uint8_t tow_f);

// Overloaded functions to convert SBP messages.
Heartbeat convertSbpMsgToRosMsg(const msg_heartbeat_t& sbp_msg);
ImuRaw convertSbpMsgToRosMsg(const msg_imu_raw_t& sbp_msg);

}  // namespace piksi_multi_msgs

#endif  // PIKSI_MULTI_MSGS_CONVERSIONS_H_
