#ifndef PIKSI_MULTI_MSGS_CONVERSIONS_H_
#define PIKSI_MULTI_MSGS_CONVERSIONS_H_

#include "piksi_multi_msgs/ExtEvent.h"
#include "piksi_multi_msgs/GpsTow.h"
#include "piksi_multi_msgs/Heartbeat.h"
#include "piksi_multi_msgs/ImuAux.h"
#include "piksi_multi_msgs/ImuRaw.h"
#include "piksi_multi_msgs/Vector3Int.h"

#include <libsbp/ext_events.h>
#include <libsbp/imu.h>
#include <libsbp/system.h>

namespace piksi_multi_msgs {
// Functions to create specific ROS message types.
Gpio convertSbpGpioToRos(const uint8_t pin, const bool value);
GpsTimeValue convertSbpGpsTimeValueToRos(const uint16_t wn, const uint32_t tow,
                                         const int32_t ns_residual);
GpsTow convertSbpGpsTowToRos(const uint32_t tow, const uint8_t tow_f);
Vector3Int convertSbpVector3IntToRos(const int16_t x, const int16_t y,
                                     const int16_t z);

// Overloaded functions to convert SBP messages. Sorted by SBP documentation
// occurance.
ExtEvent convertSbpMsgToRosMsg(const msg_ext_event_t& sbp_msg);
ImuRaw convertSbpMsgToRosMsg(const msg_imu_raw_t& sbp_msg);
ImuAux convertSbpMsgToRosMsg(const msg_imu_aux_t& sbp_msg);
Heartbeat convertSbpMsgToRosMsg(const msg_heartbeat_t& sbp_msg);

}  // namespace piksi_multi_msgs

#endif  // PIKSI_MULTI_MSGS_CONVERSIONS_H_
