#ifndef PIKSI_MULTI_MSGS_CONVERSIONS_H_
#define PIKSI_MULTI_MSGS_CONVERSIONS_H_

#include "piksi_multi_msgs/AccuracyQuaternion.h"
#include "piksi_multi_msgs/AccuracyTangentPlane.h"
#include "piksi_multi_msgs/AcqResult.h"
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
#include "piksi_multi_msgs/GpsObservation.h"
#include "piksi_multi_msgs/GpsTime.h"
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
#include "piksi_multi_msgs/PosEcefCov.h"
#include "piksi_multi_msgs/PosLlh.h"
#include "piksi_multi_msgs/PosLlhCov.h"
#include "piksi_multi_msgs/PseudoRange.h"
#include "piksi_multi_msgs/QuaternionInt.h"
#include "piksi_multi_msgs/ResetFiltersValue.h"
#include "piksi_multi_msgs/ResetValue.h"
#include "piksi_multi_msgs/SatelliteIdentifier.h"
#include "piksi_multi_msgs/SbasRaw.h"
#include "piksi_multi_msgs/Startup.h"
#include "piksi_multi_msgs/Statistics.h"
#include "piksi_multi_msgs/TrackingState.h"
#include "piksi_multi_msgs/TrackingStates.h"
#include "piksi_multi_msgs/Uart.h"
#include "piksi_multi_msgs/UartState.h"
#include "piksi_multi_msgs/UtcTime.h"
#include "piksi_multi_msgs/Vector3Int.h"
#include "piksi_multi_msgs/Vector3Int32.h"
#include "piksi_multi_msgs/VectorBody.h"
#include "piksi_multi_msgs/VectorEcef.h"
#include "piksi_multi_msgs/VectorNed.h"
#include "piksi_multi_msgs/VelBody.h"
#include "piksi_multi_msgs/VelEcef.h"
#include "piksi_multi_msgs/VelEcefCov.h"
#include "piksi_multi_msgs/VelNed.h"
#include "piksi_multi_msgs/VelNedCov.h"
#include "piksi_multi_msgs/VelocityMode.h"

#include "geometry_msgs/Vector3.h"

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
AccuracyTangentPlane convertSbpAccuracyTangentPlaneToRos(
    const uint16_t h_accuracy, const uint16_t v_accuracy);
AccuracyQuaternion convertSbpAccuracyQuaternionToRos(const float w_accuracy,
                                                     const float x_accuracy,
                                                     const float y_accuracy,
                                                     const float z_accuracy);
QuaternionInt convertSbpOrientationQuatToRos(const int32_t w, const int32_t x,
                                             const int32_t y, const int32_t z);
CovCartesian convertSbpCovCartesianToRos(const float x_x, const float x_y,
                                         const float x_z, const float y_y,
                                         const float y_z, const float z_z);
CovTangentNed convertSbpCovTangentNedToRos(const float n_n, const float n_e,
                                           const float n_d, const float e_e,
                                           const float e_d, const float d_d);

// GPS observation.
GpsObservation convertSbpGpsObservationToRos(
    const PseudoRange& pseudo_range,
    const CarrierPhaseCycles& carrier_phase_cycles, const Doppler& doppler,
    const CarrierToNoise& cn, const uint8_t lock, const bool raim_exclusion,
    const SatelliteIdentifier& sid);
PseudoRange convertSbpPseudoRangeToRos(const uint32_t range, const bool valid);
CarrierPhaseCycles convertSbpCarrierPhaseCyclesToRos(
    const int32_t i, const int32_t f, const bool valid,
    const bool half_cycle_ambiguity_resolved);
Doppler convertSbpDopplerToRos(const int32_t i, const int32_t f,
                               const bool valid);
CarrierToNoise convertSbpCarrierToNoiseToRos(const uint8_t density);
SatelliteIdentifier convertSbpSatelliteIdentifierToRos(const uint8_t sat,
                                                       const uint8_t code);

GpsTimeValue convertSbpGpsTimeValueToRos(const uint16_t wn, const uint32_t tow,
                                         const int32_t ns_residual);
GpsTow convertSbpGpsTowToRos(const uint32_t tow, const uint8_t tow_f);
geometry_msgs::Point convertSbpPointToRos(const double x, const double y,
                                          const double z);
PointWgs84 convertSbpPointWgs84ToRos(const double lat, const double lon,
                                     const double height);
Uart convertSbpUartToRos(const float tx_throughput, const float rx_throughput,
                         const uint16_t crc_error_count,
                         const uint16_t io_error_count,
                         const uint8_t tx_buffer_level,
                         const uint8_t rx_buffer_level);
Statistics convertSbpStatisticsToRos(const int32_t avg, const int32_t min,
                                     const int32_t max, const int32_t current);
Vector3Int convertSbpVector3IntToRos(const int16_t x, const int16_t y,
                                     const int16_t z);
Vector3Int32 convertSbpVector3Int32ToRos(const int32_t x, const int32_t y,
                                         const int32_t z);
VectorNed convertSbpVectorNedToRos(const int32_t n, const int32_t e,
                                   const int32_t d);

// Overloaded functions to convert SBP messages to ROS msgs. Sorted by SBP
// documentation occurance.
// TODO(rikba): Implement all commented out.
// Ext Events
ExtEvent convertSbpMsgToRosMsg(const msg_ext_event_t& sbp_msg);
// Imu
ImuRaw convertSbpMsgToRosMsg(const msg_imu_raw_t& sbp_msg);
ImuAux convertSbpMsgToRosMsg(const msg_imu_aux_t& sbp_msg);
// Logging
Log convertSbpMsgToRosMsg(const msg_log_t& sbp_msg);
Fwd convertSbpMsgToRosMsg(const msg_fwd_t& sbp_msg);
// Mag
MagRaw convertSbpMsgToRosMsg(const msg_mag_raw_t& sbp_msg);
// Navigation
GpsTime convertSbpMsgToRosMsg(const msg_gps_time_t& sbp_msg);
UtcTime convertSbpMsgToRosMsg(const msg_utc_time_t& sbp_msg);
Dops convertSbpMsgToRosMsg(const msg_dops_t& sbp_msg);
PosEcef convertSbpMsgToRosMsg(const msg_pos_ecef_t& sbp_msg);
PosEcefCov convertSbpMsgToRosMsg(const msg_pos_ecef_cov_t& sbp_msg);
PosLlh convertSbpMsgToRosMsg(const msg_pos_llh_t& sbp_msg);
PosLlhCov convertSbpMsgToRosMsg(const msg_pos_llh_cov_t& sbp_msg);
BaselineEcef convertSbpMsgToRosMsg(const msg_baseline_ecef_t& sbp_msg);
BaselineNed convertSbpMsgToRosMsg(const msg_baseline_ned_t& sbp_msg);
VelEcef convertSbpMsgToRosMsg(const msg_vel_ecef_t& sbp_msg);
VelEcefCov convertSbpMsgToRosMsg(const msg_vel_ecef_cov_t& sbp_msg);
VelNed convertSbpMsgToRosMsg(const msg_vel_ned_t& sbp_msg);
VelNedCov convertSbpMsgToRosMsg(const msg_vel_ned_cov_t& sbp_msg);
VelBody convertSbpMsgToRosMsg(const msg_vel_body_t& sbp_msg);
AgeCorrections convertSbpMsgToRosMsg(const msg_age_corrections_t& sbp_msg);
// Observation
Obs convertSbpMsgToRosMsg(const msg_obs_t& sbp_msg);
PointWgs84 convertSbpMsgToRosMsg(const msg_base_pos_llh_t& sbp_msg);
geometry_msgs::Point convertSbpMsgToRosMsg(const msg_base_pos_ecef_t& sbp_msg);
// EphemerisGps convertSbpMsgToRosMsg(const msg_ephemeris_gps_t& sbp_msg);
// EphemerisQzss convertSbpMsgToRosMsg(const msg_ephemeris_qzss_t& sbp_msg);
// EphemerisBds convertSbpMsgToRosMsg(const msg_ephemeris_bds_t& sbp_msg);
// EphemerisGal convertSbpMsgToRosMsg(const msg_ephemeris_gal_t& sbp_msg);
// EphemerisSbas convertSbpMsgToRosMsg(const msg_ephemeris_sbas_t& sbp_msg);
// EphemerisGlo convertSbpMsgToRosMsg(const msg_ephemeris_glo_t& sbp_msg);
// Iono convertSbpMsgToRosMsg(const msg_iono_t& sbp_msg);
// GnssCapb convertSbpMsgToRosMsg(const msg_gnss_capb_t& sbp_msg);
// GroupDelay convertSbpMsgToRosMsg(const msg_group_delay_t& sbp_msg);
// AlmanacGps convertSbpMsgToRosMsg(const msg_almanac_gps_t& sbp_msg);
// AlmanacGlo convertSbpMsgToRosMsg(const msg_almanac_glo_t& sbp_msg);
// GloBiases convertSbpMsgToRosMsg(const msg_glo_biases_t& sbp_msg);
// SvAzEl convertSbpMsgToRosMsg(const msg_sv_az_el_t& sbp_msg);
// Osr convertSbpMsgToRosMsg(const msg_osr_t& sbp_msg);
// System
Startup convertSbpMsgToRosMsg(const msg_startup_t& sbp_msg);
DgnssStatus convertSbpMsgToRosMsg(const msg_dgnss_status_t& sbp_msg);
Heartbeat convertSbpMsgToRosMsg(const msg_heartbeat_t& sbp_msg);
InsStatus convertSbpMsgToRosMsg(const msg_ins_status_t& sbp_msg);
// Acquisition
AcqResult convertSbpMsgToRosMsg(const msg_acq_result_t& sbp_msg);
// AcqSvProfile convertSbpMsgToRosMsg(const msg_acq_sv_profile_t& sbp_msg);
// File IO
// Linux
CpuState convertSbpMsgToRosMsg(const msg_linux_cpu_state_t& sbp_msg);
MemState convertSbpMsgToRosMsg(const msg_linux_mem_state_t& sbp_msg);
// SysState convertSbpMsgToRosMsg(const msg_linux_sys_state_t& sbp_msg);
// ProcessSocketCounts convertSbpMsgToRosMsg(const
// msg_linux_process_socket_counts_t& sbp_msg); ProcessSocketQueues
// convertSbpMsgToRosMsg(const msg_linux_process_socket_queues_t& sbp_msg);
// SocketUsage convertSbpMsgToRosMsg(const msg_linux_socket_usage_t& sbp_msg);
// ProcessFdCount convertSbpMsgToRosMsg(const msg_linux_process_fd_count_t&
// sbp_msg); ProcessFdSummary convertSbpMsgToRosMsg(const
// msg_linux_process_fd_summary_t& sbp_msg);
// Orientation
BaselineHeading convertSbpMsgToRosMsg(const msg_baseline_heading_t& sbp_msg);
OrientationQuat convertSbpMsgToRosMsg(const msg_orient_quat_t& sbp_msg);
// OrientationEuler convertSbpMsgToRosMsg(const msg_orient_euler_t& sbp_msg);
// AngularRate convertSbpMsgToRosMsg(const msg_angular_rate_t& sbp_msg);
// Piksi
// ThreadState convertSbpMsgToRosMsg(const msg_thread_state_t& sbp_msg);
UartState convertSbpMsgToRosMsg(const msg_uart_state_t& sbp_msg);
// IarState convertSbpMsgToRosMsg(const msg_iar_state_t& sbp_msg);
DeviceMonitor convertSbpMsgToRosMsg(const msg_device_monitor_t& sbp_msg);
// CommandResp convertSbpMsgToRosMsg(const msg_command_resp_t& sbp_msg);
// CommandOutput convertSbpMsgToRosMsg(const msg_command_output_t& sbp_msg);
// NetworkStateResp convertSbpMsgToRosMsg(const msg_network_state_resp_t&
// sbp_msg); NetworkBandwidthUsage convertSbpMsgToRosMsg(const
// msg_network_bandwidth_usage_t& sbp_msg); CellModemStatus
// convertSbpMsgToRosMsg(const msg_cell_modem_status_t& sbp_msg); Specan
// convertSbpMsgToRosMsg(const msg_specan_t& sbp_msg); FrontEndGain
// convertSbpMsgToRosMsg(const msg_front_end_gain_t& sbp_msg);
// Sbas
SbasRaw convertSbpMsgToRosMsg(const msg_sbas_raw_t& sbp_msg);
// Ssr
// SsrRaw convertSbpMsgToRosMsg(const msg_ssr_raw_t& sbp_msg);
// SsrCodeBiases convertSbpMsgToRosMsg(const msg_ssr_code_biases_t& sbp_msg);
// SsrPhaseBiases convertSbpMsgToRosMsg(const msg_ssr_phase_biases_t& sbp_msg);
// SsrStecCorrection convertSbpMsgToRosMsg(const msg_ssr_stec_correction_t&
// sbp_msg); SsrGriddedCorrection convertSbpMsgToRosMsg(const
// msg_ssr_gridded_correction_t& sbp_msg); SsrGridDefinition
// convertSbpMsgToRosMsg(const msg_ssr_grid_definition_t& sbp_msg);
// Tracking
TrackingStates convertSbpMsgToRosMsg(const msg_tracking_state_t& sbp_msg);
MeasurementStates convertSbpMsgToRosMsg(const msg_measurement_state_t& sbp_msg);
// TrackingIq convertSbpMsgToRosMsg(const msg_tracking_iq_t& sbp_msg);
// User
// UserData convertSbpMsgToRosMsg(const msg_user_data_t& sbp_msg);
// Vehicle
// Odometry convertSbpMsgToRosMsg(const msg_odometry_t& sbp_msg);

// Overloaded functions to convert ROS messages to SBP msgs. Sorted by SBP
// documentation occurance.
// TODO(rikba): Implement all messages that are useful to set settings on piksi.

}  // namespace piksi_multi_msgs

#endif  // PIKSI_MULTI_MSGS_CONVERSIONS_H_
