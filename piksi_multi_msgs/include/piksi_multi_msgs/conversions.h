#ifndef PIKSI_MULTI_MSGS_CONVERSIONS_H_
#define PIKSI_MULTI_MSGS_CONVERSIONS_H_

#include "piksi_multi_msgs/AccuracyQuaternion.h"
#include "piksi_multi_msgs/AccuracyTangentPlane.h"
#include "piksi_multi_msgs/Acquisation.h"
#include "piksi_multi_msgs/AgeCorrections.h"
#include "piksi_multi_msgs/BaselineEcef.h"
#include "piksi_multi_msgs/BaselineHeading.h"
#include "piksi_multi_msgs/BaselineNed.h"
#include "piksi_multi_msgs/CarrierPhaseCycles.h"
#include "piksi_multi_msgs/CarrierToNoise.h"
#include "piksi_multi_msgs/CovCartesian.h"
#include "piksi_multi_msgs/CovTangentNed.h"
#include "piksi_multi_msgs/CpuState.h"
#include "piksi_multi_msgs/DeviceMonitor.h"
#include "piksi_multi_msgs/DgnssStatus.h"
#include "piksi_multi_msgs/Doppler.h"
#include "piksi_multi_msgs/Dops.h"
#include "piksi_multi_msgs/ExtEvent.h"
#include "piksi_multi_msgs/FixMode.h"
#include "piksi_multi_msgs/Fwd.h"
#include "piksi_multi_msgs/Gpio.h"
#include "piksi_multi_msgs/GpsObservation.h"
#include "piksi_multi_msgs/GpsTime.h"
#include "piksi_multi_msgs/GpsTimeSource.h"
#include "piksi_multi_msgs/GpsTimeValue.h"
#include "piksi_multi_msgs/GpsTow.h"
#include "piksi_multi_msgs/Heartbeat.h"
#include "piksi_multi_msgs/ImuAux.h"
#include "piksi_multi_msgs/ImuRaw.h"
#include "piksi_multi_msgs/InertialNavigationMode.h"
#include "piksi_multi_msgs/InsStatus.h"
#include "piksi_multi_msgs/Log.h"
#include "piksi_multi_msgs/MagRaw.h"
#include "piksi_multi_msgs/MeasurementState.h"
#include "piksi_multi_msgs/MeasurementStates.h"
#include "piksi_multi_msgs/MemState.h"
#include "piksi_multi_msgs/Obs.h"
#include "piksi_multi_msgs/OrientationQuat.h"
#include "piksi_multi_msgs/PointEcef.h"
#include "piksi_multi_msgs/PointNed.h"
#include "piksi_multi_msgs/PointWgs84.h"
#include "piksi_multi_msgs/PosEcef.h"
#include "piksi_multi_msgs/UtcTime.h"
#include "piksi_multi_msgs/Vector3Int.h"

#include <libsbp/acquisition.h>
#include <libsbp/ext_events.h>
#include <libsbp/imu.h>
#include <libsbp/linux.h>
#include <libsbp/logging.h>
#include <libsbp/mag.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>
#include <libsbp/piksi.h>
#include <libsbp/sbas.h>
#include <libsbp/ssr.h>
#include <libsbp/system.h>
#include <libsbp/tracking.h>
#include <libsbp/user.h>
#include <libsbp/vehicle.h>

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
Log convertSbpMsgToRosMsg(const msg_log_t& sbp_msg);
Fwd convertSbpMsgToRosMsg(const msg_fwd_t& sbp_msg);
MagRaw convertSbpMsgToRosMsg(const msg_mag_raw_t& sbp_msg);
GpsTime convertSbpMsgToRosMsg(const msg_gps_time_t& sbp_msg);
UtcTime convertSbpMsgToRosMsg(const msg_utc_time_t& sbp_msg);
Dops convertSbpMsgToRosMsg(const msg_dops_t& sbp_msg);
PosEcef convertSbpMsgToRosMsg(const msg_pos_ecef_t& sbp_msg);

}  // namespace piksi_multi_msgs

#endif  // PIKSI_MULTI_MSGS_CONVERSIONS_H_
