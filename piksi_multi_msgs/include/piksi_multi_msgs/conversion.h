/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

// This message is automatically generated using generator.py
// PLEASE DO NOT MODIFY MANUALLY.

#ifndef PIKSI_MULTI_MSGS_CONVERSION_H_
#define PIKSI_MULTI_MSGS_CONVERSION_H_

// Include all SBP headers.
#include <libsbp/acquisition.h>
#include <libsbp/bootload.h>
#include <libsbp/ext_events.h>
#include <libsbp/file_io.h>
#include <libsbp/flash.h>
#include <libsbp/gnss.h>
#include <libsbp/imu.h>
#include <libsbp/linux.h>
#include <libsbp/logging.h>
#include <libsbp/mag.h>
#include <libsbp/navigation.h>
#include <libsbp/ndb.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>
#include <libsbp/piksi.h>
#include <libsbp/sbas.h>
#include <libsbp/settings.h>
#include <libsbp/ssr.h>
#include <libsbp/system.h>
#include <libsbp/tracking.h>
#include <libsbp/user.h>
#include <libsbp/vehicle.h>

// Include all generated SBP ROS messages.
#include <piksi_multi_msgs/MsgAcqResult.h>
#include <piksi_multi_msgs/AcqSvProfile.h>
#include <piksi_multi_msgs/MsgAcqSvProfile.h>
#include <piksi_multi_msgs/MsgBootloaderHandshakeReq.h>
#include <piksi_multi_msgs/MsgBootloaderHandshakeResp.h>
#include <piksi_multi_msgs/MsgBootloaderJumpToApp.h>
#include <piksi_multi_msgs/MsgNapDeviceDnaReq.h>
#include <piksi_multi_msgs/MsgNapDeviceDnaResp.h>
#include <piksi_multi_msgs/MsgExtEvent.h>
#include <piksi_multi_msgs/MsgFileioReadReq.h>
#include <piksi_multi_msgs/MsgFileioReadResp.h>
#include <piksi_multi_msgs/MsgFileioReadDirReq.h>
#include <piksi_multi_msgs/MsgFileioReadDirResp.h>
#include <piksi_multi_msgs/MsgFileioRemove.h>
#include <piksi_multi_msgs/MsgFileioWriteReq.h>
#include <piksi_multi_msgs/MsgFileioWriteResp.h>
#include <piksi_multi_msgs/MsgFileioConfigReq.h>
#include <piksi_multi_msgs/MsgFileioConfigResp.h>
#include <piksi_multi_msgs/MsgFlashProgram.h>
#include <piksi_multi_msgs/MsgFlashDone.h>
#include <piksi_multi_msgs/MsgFlashReadReq.h>
#include <piksi_multi_msgs/MsgFlashReadResp.h>
#include <piksi_multi_msgs/MsgFlashErase.h>
#include <piksi_multi_msgs/MsgStmFlashLockSector.h>
#include <piksi_multi_msgs/MsgStmFlashUnlockSector.h>
#include <piksi_multi_msgs/MsgStmUniqueIdReq.h>
#include <piksi_multi_msgs/MsgStmUniqueIdResp.h>
#include <piksi_multi_msgs/MsgM25FlashWriteStatus.h>
#include <piksi_multi_msgs/GnssSignal.h>
#include <piksi_multi_msgs/SvId.h>
#include <piksi_multi_msgs/GpsTimeSec.h>
#include <piksi_multi_msgs/GpsTime.h>
#include <piksi_multi_msgs/CarrierPhase.h>
#include <piksi_multi_msgs/MsgImuRaw.h>
#include <piksi_multi_msgs/MsgImuAux.h>
#include <piksi_multi_msgs/MsgLinuxCpuState.h>
#include <piksi_multi_msgs/MsgLinuxMemState.h>
#include <piksi_multi_msgs/MsgLinuxSysState.h>
#include <piksi_multi_msgs/MsgLinuxProcessSocketCounts.h>
#include <piksi_multi_msgs/MsgLinuxProcessSocketQueues.h>
#include <piksi_multi_msgs/MsgLinuxSocketUsage.h>
#include <piksi_multi_msgs/MsgLinuxProcessFdCount.h>
#include <piksi_multi_msgs/MsgLinuxProcessFdSummary.h>
#include <piksi_multi_msgs/MsgLog.h>
#include <piksi_multi_msgs/MsgFwd.h>
#include <piksi_multi_msgs/MsgMagRaw.h>
#include <piksi_multi_msgs/MsgGpsTime.h>
#include <piksi_multi_msgs/MsgUtcTime.h>
#include <piksi_multi_msgs/MsgDops.h>
#include <piksi_multi_msgs/MsgPosEcef.h>
#include <piksi_multi_msgs/MsgPosEcefCov.h>
#include <piksi_multi_msgs/MsgPosLlh.h>
#include <piksi_multi_msgs/MsgPosLlhCov.h>
#include <piksi_multi_msgs/MsgBaselineEcef.h>
#include <piksi_multi_msgs/MsgBaselineNed.h>
#include <piksi_multi_msgs/MsgVelEcef.h>
#include <piksi_multi_msgs/MsgVelEcefCov.h>
#include <piksi_multi_msgs/MsgVelNed.h>
#include <piksi_multi_msgs/MsgVelNedCov.h>
#include <piksi_multi_msgs/MsgVelBody.h>
#include <piksi_multi_msgs/MsgAgeCorrections.h>
#include <piksi_multi_msgs/MsgNdbEvent.h>
#include <piksi_multi_msgs/ObservationHeader.h>
#include <piksi_multi_msgs/Doppler.h>
#include <piksi_multi_msgs/PackedObsContent.h>
#include <piksi_multi_msgs/PackedOsrContent.h>
#include <piksi_multi_msgs/MsgObs.h>
#include <piksi_multi_msgs/MsgBasePosLlh.h>
#include <piksi_multi_msgs/MsgBasePosEcef.h>
#include <piksi_multi_msgs/EphemerisCommonContent.h>
#include <piksi_multi_msgs/MsgEphemerisGps.h>
#include <piksi_multi_msgs/MsgEphemerisQzss.h>
#include <piksi_multi_msgs/MsgEphemerisBds.h>
#include <piksi_multi_msgs/MsgEphemerisGal.h>
#include <piksi_multi_msgs/MsgEphemerisSbas.h>
#include <piksi_multi_msgs/MsgEphemerisGlo.h>
#include <piksi_multi_msgs/MsgIono.h>
#include <piksi_multi_msgs/GnssCapb.h>
#include <piksi_multi_msgs/MsgGnssCapb.h>
#include <piksi_multi_msgs/MsgGroupDelay.h>
#include <piksi_multi_msgs/AlmanacCommonContent.h>
#include <piksi_multi_msgs/MsgAlmanacGps.h>
#include <piksi_multi_msgs/MsgAlmanacGlo.h>
#include <piksi_multi_msgs/MsgGloBiases.h>
#include <piksi_multi_msgs/SvAzEl.h>
#include <piksi_multi_msgs/MsgSvAzEl.h>
#include <piksi_multi_msgs/MsgOsr.h>
#include <piksi_multi_msgs/MsgBaselineHeading.h>
#include <piksi_multi_msgs/MsgOrientQuat.h>
#include <piksi_multi_msgs/MsgOrientEuler.h>
#include <piksi_multi_msgs/MsgAngularRate.h>
#include <piksi_multi_msgs/MsgSetTime.h>
#include <piksi_multi_msgs/MsgReset.h>
#include <piksi_multi_msgs/MsgResetFilters.h>
#include <piksi_multi_msgs/MsgThreadState.h>
#include <piksi_multi_msgs/UartChannel.h>
#include <piksi_multi_msgs/Period.h>
#include <piksi_multi_msgs/Latency.h>
#include <piksi_multi_msgs/MsgUartState.h>
#include <piksi_multi_msgs/MsgIarState.h>
#include <piksi_multi_msgs/MsgMaskSatellite.h>
#include <piksi_multi_msgs/MsgDeviceMonitor.h>
#include <piksi_multi_msgs/MsgCommandReq.h>
#include <piksi_multi_msgs/MsgCommandResp.h>
#include <piksi_multi_msgs/MsgCommandOutput.h>
#include <piksi_multi_msgs/MsgNetworkStateReq.h>
#include <piksi_multi_msgs/MsgNetworkStateResp.h>
#include <piksi_multi_msgs/NetworkUsage.h>
#include <piksi_multi_msgs/MsgNetworkBandwidthUsage.h>
#include <piksi_multi_msgs/MsgCellModemStatus.h>
#include <piksi_multi_msgs/MsgSpecan.h>
#include <piksi_multi_msgs/MsgFrontEndGain.h>
#include <piksi_multi_msgs/MsgSbasRaw.h>
#include <piksi_multi_msgs/MsgSettingsSave.h>
#include <piksi_multi_msgs/MsgSettingsWrite.h>
#include <piksi_multi_msgs/MsgSettingsWriteResp.h>
#include <piksi_multi_msgs/MsgSettingsReadReq.h>
#include <piksi_multi_msgs/MsgSettingsReadResp.h>
#include <piksi_multi_msgs/MsgSettingsReadByIndexReq.h>
#include <piksi_multi_msgs/MsgSettingsReadByIndexResp.h>
#include <piksi_multi_msgs/MsgSettingsReadByIndexDone.h>
#include <piksi_multi_msgs/MsgSettingsRegister.h>
#include <piksi_multi_msgs/MsgSettingsRegisterResp.h>
#include <piksi_multi_msgs/CodeBiasesContent.h>
#include <piksi_multi_msgs/PhaseBiasesContent.h>
#include <piksi_multi_msgs/StecHeader.h>
#include <piksi_multi_msgs/GriddedCorrectionHeader.h>
#include <piksi_multi_msgs/StecSatElement.h>
#include <piksi_multi_msgs/TroposphericDelayCorrection.h>
#include <piksi_multi_msgs/StecResidual.h>
#include <piksi_multi_msgs/GridElement.h>
#include <piksi_multi_msgs/GridDefinitionHeader.h>
#include <piksi_multi_msgs/MsgSsrOrbitClock.h>
#include <piksi_multi_msgs/MsgSsrCodeBiases.h>
#include <piksi_multi_msgs/MsgSsrPhaseBiases.h>
#include <piksi_multi_msgs/MsgSsrStecCorrection.h>
#include <piksi_multi_msgs/MsgSsrGriddedCorrection.h>
#include <piksi_multi_msgs/MsgSsrGridDefinition.h>
#include <piksi_multi_msgs/MsgStartup.h>
#include <piksi_multi_msgs/MsgDgnssStatus.h>
#include <piksi_multi_msgs/MsgHeartbeat.h>
#include <piksi_multi_msgs/MsgInsStatus.h>
#include <piksi_multi_msgs/MsgCsacTelemetry.h>
#include <piksi_multi_msgs/MsgCsacTelemetryLabels.h>
#include <piksi_multi_msgs/TrackingChannelState.h>
#include <piksi_multi_msgs/MsgTrackingState.h>
#include <piksi_multi_msgs/MeasurementState.h>
#include <piksi_multi_msgs/MsgMeasurementState.h>
#include <piksi_multi_msgs/TrackingChannelCorrelation.h>
#include <piksi_multi_msgs/MsgTrackingIq.h>
#include <piksi_multi_msgs/MsgUserData.h>
#include <piksi_multi_msgs/MsgOdometry.h>

// Autocreated conversions.
namespace piksi_multi_msgs {
MsgAcqResult convertSbpMsgToRosMsg(const msg_acq_result_t& sbp_msg);
AcqSvProfile convertSbpMsgToRosMsg(const acq_sv_profile_t& sbp_msg);
MsgAcqSvProfile convertSbpMsgToRosMsg(const msg_acq_sv_profile_t& sbp_msg);
MsgBootloaderHandshakeResp convertSbpMsgToRosMsg(const msg_bootloader_handshake_resp_t& sbp_msg);
MsgBootloaderJumpToApp convertSbpMsgToRosMsg(const msg_bootloader_jump_to_app_t& sbp_msg);
MsgNapDeviceDnaResp convertSbpMsgToRosMsg(const msg_nap_device_dna_resp_t& sbp_msg);
MsgExtEvent convertSbpMsgToRosMsg(const msg_ext_event_t& sbp_msg);
MsgFileioReadReq convertSbpMsgToRosMsg(const msg_fileio_read_req_t& sbp_msg);
MsgFileioReadResp convertSbpMsgToRosMsg(const msg_fileio_read_resp_t& sbp_msg);
MsgFileioReadDirReq convertSbpMsgToRosMsg(const msg_fileio_read_dir_req_t& sbp_msg);
MsgFileioReadDirResp convertSbpMsgToRosMsg(const msg_fileio_read_dir_resp_t& sbp_msg);
MsgFileioRemove convertSbpMsgToRosMsg(const msg_fileio_remove_t& sbp_msg);
MsgFileioWriteReq convertSbpMsgToRosMsg(const msg_fileio_write_req_t& sbp_msg);
MsgFileioWriteResp convertSbpMsgToRosMsg(const msg_fileio_write_resp_t& sbp_msg);
MsgFileioConfigReq convertSbpMsgToRosMsg(const msg_fileio_config_req_t& sbp_msg);
MsgFileioConfigResp convertSbpMsgToRosMsg(const msg_fileio_config_resp_t& sbp_msg);
MsgFlashProgram convertSbpMsgToRosMsg(const msg_flash_program_t& sbp_msg);
MsgFlashDone convertSbpMsgToRosMsg(const msg_flash_done_t& sbp_msg);
MsgFlashReadReq convertSbpMsgToRosMsg(const msg_flash_read_req_t& sbp_msg);
MsgFlashReadResp convertSbpMsgToRosMsg(const msg_flash_read_resp_t& sbp_msg);
MsgFlashErase convertSbpMsgToRosMsg(const msg_flash_erase_t& sbp_msg);
MsgStmFlashLockSector convertSbpMsgToRosMsg(const msg_stm_flash_lock_sector_t& sbp_msg);
MsgStmFlashUnlockSector convertSbpMsgToRosMsg(const msg_stm_flash_unlock_sector_t& sbp_msg);
MsgStmUniqueIdResp convertSbpMsgToRosMsg(const msg_stm_unique_id_resp_t& sbp_msg);
MsgM25FlashWriteStatus convertSbpMsgToRosMsg(const msg_m25_flash_write_status_t& sbp_msg);
GnssSignal convertSbpMsgToRosMsg(const sbp_gnss_signal_t& sbp_msg);
SvId convertSbpMsgToRosMsg(const sv_id_t& sbp_msg);
GpsTimeSec convertSbpMsgToRosMsg(const gps_time_sec_t& sbp_msg);
GpsTime convertSbpMsgToRosMsg(const sbp_gps_time_t& sbp_msg);
CarrierPhase convertSbpMsgToRosMsg(const carrier_phase_t& sbp_msg);
MsgImuRaw convertSbpMsgToRosMsg(const msg_imu_raw_t& sbp_msg);
MsgImuAux convertSbpMsgToRosMsg(const msg_imu_aux_t& sbp_msg);
MsgLinuxCpuState convertSbpMsgToRosMsg(const msg_linux_cpu_state_t& sbp_msg);
MsgLinuxMemState convertSbpMsgToRosMsg(const msg_linux_mem_state_t& sbp_msg);
MsgLinuxSysState convertSbpMsgToRosMsg(const msg_linux_sys_state_t& sbp_msg);
MsgLinuxProcessSocketCounts convertSbpMsgToRosMsg(const msg_linux_process_socket_counts_t& sbp_msg);
MsgLinuxProcessSocketQueues convertSbpMsgToRosMsg(const msg_linux_process_socket_queues_t& sbp_msg);
MsgLinuxSocketUsage convertSbpMsgToRosMsg(const msg_linux_socket_usage_t& sbp_msg);
MsgLinuxProcessFdCount convertSbpMsgToRosMsg(const msg_linux_process_fd_count_t& sbp_msg);
MsgLinuxProcessFdSummary convertSbpMsgToRosMsg(const msg_linux_process_fd_summary_t& sbp_msg);
MsgLog convertSbpMsgToRosMsg(const msg_log_t& sbp_msg);
MsgFwd convertSbpMsgToRosMsg(const msg_fwd_t& sbp_msg);
MsgMagRaw convertSbpMsgToRosMsg(const msg_mag_raw_t& sbp_msg);
MsgGpsTime convertSbpMsgToRosMsg(const msg_gps_time_t& sbp_msg);
MsgUtcTime convertSbpMsgToRosMsg(const msg_utc_time_t& sbp_msg);
MsgDops convertSbpMsgToRosMsg(const msg_dops_t& sbp_msg);
MsgPosEcef convertSbpMsgToRosMsg(const msg_pos_ecef_t& sbp_msg);
MsgPosEcefCov convertSbpMsgToRosMsg(const msg_pos_ecef_cov_t& sbp_msg);
MsgPosLlh convertSbpMsgToRosMsg(const msg_pos_llh_t& sbp_msg);
MsgPosLlhCov convertSbpMsgToRosMsg(const msg_pos_llh_cov_t& sbp_msg);
MsgBaselineEcef convertSbpMsgToRosMsg(const msg_baseline_ecef_t& sbp_msg);
MsgBaselineNed convertSbpMsgToRosMsg(const msg_baseline_ned_t& sbp_msg);
MsgVelEcef convertSbpMsgToRosMsg(const msg_vel_ecef_t& sbp_msg);
MsgVelEcefCov convertSbpMsgToRosMsg(const msg_vel_ecef_cov_t& sbp_msg);
MsgVelNed convertSbpMsgToRosMsg(const msg_vel_ned_t& sbp_msg);
MsgVelNedCov convertSbpMsgToRosMsg(const msg_vel_ned_cov_t& sbp_msg);
MsgVelBody convertSbpMsgToRosMsg(const msg_vel_body_t& sbp_msg);
MsgAgeCorrections convertSbpMsgToRosMsg(const msg_age_corrections_t& sbp_msg);
MsgNdbEvent convertSbpMsgToRosMsg(const msg_ndb_event_t& sbp_msg);
ObservationHeader convertSbpMsgToRosMsg(const observation_header_t& sbp_msg);
Doppler convertSbpMsgToRosMsg(const doppler_t& sbp_msg);
PackedObsContent convertSbpMsgToRosMsg(const packed_obs_content_t& sbp_msg);
PackedOsrContent convertSbpMsgToRosMsg(const packed_osr_content_t& sbp_msg);
MsgObs convertSbpMsgToRosMsg(const msg_obs_t& sbp_msg);
MsgBasePosLlh convertSbpMsgToRosMsg(const msg_base_pos_llh_t& sbp_msg);
MsgBasePosEcef convertSbpMsgToRosMsg(const msg_base_pos_ecef_t& sbp_msg);
EphemerisCommonContent convertSbpMsgToRosMsg(const ephemeris_common_content_t& sbp_msg);
MsgEphemerisGps convertSbpMsgToRosMsg(const msg_ephemeris_gps_t& sbp_msg);
MsgEphemerisQzss convertSbpMsgToRosMsg(const msg_ephemeris_qzss_t& sbp_msg);
MsgEphemerisBds convertSbpMsgToRosMsg(const msg_ephemeris_bds_t& sbp_msg);
MsgEphemerisGal convertSbpMsgToRosMsg(const msg_ephemeris_gal_t& sbp_msg);
MsgEphemerisSbas convertSbpMsgToRosMsg(const msg_ephemeris_sbas_t& sbp_msg);
MsgEphemerisGlo convertSbpMsgToRosMsg(const msg_ephemeris_glo_t& sbp_msg);
MsgIono convertSbpMsgToRosMsg(const msg_iono_t& sbp_msg);
GnssCapb convertSbpMsgToRosMsg(const gnss_capb_t& sbp_msg);
MsgGnssCapb convertSbpMsgToRosMsg(const msg_gnss_capb_t& sbp_msg);
MsgGroupDelay convertSbpMsgToRosMsg(const msg_group_delay_t& sbp_msg);
AlmanacCommonContent convertSbpMsgToRosMsg(const almanac_common_content_t& sbp_msg);
MsgAlmanacGps convertSbpMsgToRosMsg(const msg_almanac_gps_t& sbp_msg);
MsgAlmanacGlo convertSbpMsgToRosMsg(const msg_almanac_glo_t& sbp_msg);
MsgGloBiases convertSbpMsgToRosMsg(const msg_glo_biases_t& sbp_msg);
SvAzEl convertSbpMsgToRosMsg(const sv_az_el_t& sbp_msg);
MsgSvAzEl convertSbpMsgToRosMsg(const msg_sv_az_el_t& sbp_msg);
MsgOsr convertSbpMsgToRosMsg(const msg_osr_t& sbp_msg);
MsgBaselineHeading convertSbpMsgToRosMsg(const msg_baseline_heading_t& sbp_msg);
MsgOrientQuat convertSbpMsgToRosMsg(const msg_orient_quat_t& sbp_msg);
MsgOrientEuler convertSbpMsgToRosMsg(const msg_orient_euler_t& sbp_msg);
MsgAngularRate convertSbpMsgToRosMsg(const msg_angular_rate_t& sbp_msg);
MsgReset convertSbpMsgToRosMsg(const msg_reset_t& sbp_msg);
MsgResetFilters convertSbpMsgToRosMsg(const msg_reset_filters_t& sbp_msg);
MsgThreadState convertSbpMsgToRosMsg(const msg_thread_state_t& sbp_msg);
UartChannel convertSbpMsgToRosMsg(const uart_channel_t& sbp_msg);
Period convertSbpMsgToRosMsg(const period_t& sbp_msg);
Latency convertSbpMsgToRosMsg(const latency_t& sbp_msg);
MsgUartState convertSbpMsgToRosMsg(const msg_uart_state_t& sbp_msg);
MsgIarState convertSbpMsgToRosMsg(const msg_iar_state_t& sbp_msg);
MsgMaskSatellite convertSbpMsgToRosMsg(const msg_mask_satellite_t& sbp_msg);
MsgDeviceMonitor convertSbpMsgToRosMsg(const msg_device_monitor_t& sbp_msg);
MsgCommandReq convertSbpMsgToRosMsg(const msg_command_req_t& sbp_msg);
MsgCommandResp convertSbpMsgToRosMsg(const msg_command_resp_t& sbp_msg);
MsgCommandOutput convertSbpMsgToRosMsg(const msg_command_output_t& sbp_msg);
MsgNetworkStateResp convertSbpMsgToRosMsg(const msg_network_state_resp_t& sbp_msg);
NetworkUsage convertSbpMsgToRosMsg(const network_usage_t& sbp_msg);
MsgNetworkBandwidthUsage convertSbpMsgToRosMsg(const msg_network_bandwidth_usage_t& sbp_msg);
MsgCellModemStatus convertSbpMsgToRosMsg(const msg_cell_modem_status_t& sbp_msg);
MsgSpecan convertSbpMsgToRosMsg(const msg_specan_t& sbp_msg);
MsgFrontEndGain convertSbpMsgToRosMsg(const msg_front_end_gain_t& sbp_msg);
MsgSbasRaw convertSbpMsgToRosMsg(const msg_sbas_raw_t& sbp_msg);
MsgSettingsWrite convertSbpMsgToRosMsg(const msg_settings_write_t& sbp_msg);
MsgSettingsWriteResp convertSbpMsgToRosMsg(const msg_settings_write_resp_t& sbp_msg);
MsgSettingsReadReq convertSbpMsgToRosMsg(const msg_settings_read_req_t& sbp_msg);
MsgSettingsReadResp convertSbpMsgToRosMsg(const msg_settings_read_resp_t& sbp_msg);
MsgSettingsReadByIndexReq convertSbpMsgToRosMsg(const msg_settings_read_by_index_req_t& sbp_msg);
MsgSettingsReadByIndexResp convertSbpMsgToRosMsg(const msg_settings_read_by_index_resp_t& sbp_msg);
MsgSettingsRegister convertSbpMsgToRosMsg(const msg_settings_register_t& sbp_msg);
MsgSettingsRegisterResp convertSbpMsgToRosMsg(const msg_settings_register_resp_t& sbp_msg);
CodeBiasesContent convertSbpMsgToRosMsg(const code_biases_content_t& sbp_msg);
PhaseBiasesContent convertSbpMsgToRosMsg(const phase_biases_content_t& sbp_msg);
StecHeader convertSbpMsgToRosMsg(const stec_header_t& sbp_msg);
GriddedCorrectionHeader convertSbpMsgToRosMsg(const gridded_correction_header_t& sbp_msg);
StecSatElement convertSbpMsgToRosMsg(const stec_sat_element_t& sbp_msg);
TroposphericDelayCorrection convertSbpMsgToRosMsg(const tropospheric_delay_correction_t& sbp_msg);
StecResidual convertSbpMsgToRosMsg(const stec_residual_t& sbp_msg);
GridElement convertSbpMsgToRosMsg(const grid_element_t& sbp_msg);
GridDefinitionHeader convertSbpMsgToRosMsg(const grid_definition_header_t& sbp_msg);
MsgSsrOrbitClock convertSbpMsgToRosMsg(const msg_ssr_orbit_clock_t& sbp_msg);
MsgSsrCodeBiases convertSbpMsgToRosMsg(const msg_ssr_code_biases_t& sbp_msg);
MsgSsrPhaseBiases convertSbpMsgToRosMsg(const msg_ssr_phase_biases_t& sbp_msg);
MsgSsrStecCorrection convertSbpMsgToRosMsg(const msg_ssr_stec_correction_t& sbp_msg);
MsgSsrGriddedCorrection convertSbpMsgToRosMsg(const msg_ssr_gridded_correction_t& sbp_msg);
MsgSsrGridDefinition convertSbpMsgToRosMsg(const msg_ssr_grid_definition_t& sbp_msg);
MsgStartup convertSbpMsgToRosMsg(const msg_startup_t& sbp_msg);
MsgDgnssStatus convertSbpMsgToRosMsg(const msg_dgnss_status_t& sbp_msg);
MsgHeartbeat convertSbpMsgToRosMsg(const msg_heartbeat_t& sbp_msg);
MsgInsStatus convertSbpMsgToRosMsg(const msg_ins_status_t& sbp_msg);
MsgCsacTelemetry convertSbpMsgToRosMsg(const msg_csac_telemetry_t& sbp_msg);
MsgCsacTelemetryLabels convertSbpMsgToRosMsg(const msg_csac_telemetry_labels_t& sbp_msg);
TrackingChannelState convertSbpMsgToRosMsg(const tracking_channel_state_t& sbp_msg);
MsgTrackingState convertSbpMsgToRosMsg(const msg_tracking_state_t& sbp_msg);
MeasurementState convertSbpMsgToRosMsg(const measurement_state_t& sbp_msg);
MsgMeasurementState convertSbpMsgToRosMsg(const msg_measurement_state_t& sbp_msg);
TrackingChannelCorrelation convertSbpMsgToRosMsg(const tracking_channel_correlation_t& sbp_msg);
MsgTrackingIq convertSbpMsgToRosMsg(const msg_tracking_iq_t& sbp_msg);
MsgUserData convertSbpMsgToRosMsg(const msg_user_data_t& sbp_msg);
MsgOdometry convertSbpMsgToRosMsg(const msg_odometry_t& sbp_msg);
} // namespace piksi_multi_msgs

#endif // PIKSI_MULTI_MSGS_CONVERSION_H_