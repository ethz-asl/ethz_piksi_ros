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
MsgAcqResult convertSbpMsgToRosMsg(const msg_acq_result_t& sbp_msg, const uint8_t len = 0);
AcqSvProfile convertSbpMsgToRosMsg(const acq_sv_profile_t& sbp_msg, const uint8_t len = 0);
MsgAcqSvProfile convertSbpMsgToRosMsg(const msg_acq_sv_profile_t& sbp_msg, const uint8_t len = 0);
MsgBootloaderHandshakeResp convertSbpMsgToRosMsg(const msg_bootloader_handshake_resp_t& sbp_msg, const uint8_t len = 0);
MsgBootloaderJumpToApp convertSbpMsgToRosMsg(const msg_bootloader_jump_to_app_t& sbp_msg, const uint8_t len = 0);
MsgNapDeviceDnaResp convertSbpMsgToRosMsg(const msg_nap_device_dna_resp_t& sbp_msg, const uint8_t len = 0);
MsgExtEvent convertSbpMsgToRosMsg(const msg_ext_event_t& sbp_msg, const uint8_t len = 0);
MsgFileioReadReq convertSbpMsgToRosMsg(const msg_fileio_read_req_t& sbp_msg, const uint8_t len = 0);
MsgFileioReadResp convertSbpMsgToRosMsg(const msg_fileio_read_resp_t& sbp_msg, const uint8_t len = 0);
MsgFileioReadDirReq convertSbpMsgToRosMsg(const msg_fileio_read_dir_req_t& sbp_msg, const uint8_t len = 0);
MsgFileioReadDirResp convertSbpMsgToRosMsg(const msg_fileio_read_dir_resp_t& sbp_msg, const uint8_t len = 0);
MsgFileioRemove convertSbpMsgToRosMsg(const msg_fileio_remove_t& sbp_msg, const uint8_t len = 0);
MsgFileioWriteReq convertSbpMsgToRosMsg(const msg_fileio_write_req_t& sbp_msg, const uint8_t len = 0);
MsgFileioWriteResp convertSbpMsgToRosMsg(const msg_fileio_write_resp_t& sbp_msg, const uint8_t len = 0);
MsgFileioConfigReq convertSbpMsgToRosMsg(const msg_fileio_config_req_t& sbp_msg, const uint8_t len = 0);
MsgFileioConfigResp convertSbpMsgToRosMsg(const msg_fileio_config_resp_t& sbp_msg, const uint8_t len = 0);
MsgFlashProgram convertSbpMsgToRosMsg(const msg_flash_program_t& sbp_msg, const uint8_t len = 0);
MsgFlashDone convertSbpMsgToRosMsg(const msg_flash_done_t& sbp_msg, const uint8_t len = 0);
MsgFlashReadReq convertSbpMsgToRosMsg(const msg_flash_read_req_t& sbp_msg, const uint8_t len = 0);
MsgFlashReadResp convertSbpMsgToRosMsg(const msg_flash_read_resp_t& sbp_msg, const uint8_t len = 0);
MsgFlashErase convertSbpMsgToRosMsg(const msg_flash_erase_t& sbp_msg, const uint8_t len = 0);
MsgStmFlashLockSector convertSbpMsgToRosMsg(const msg_stm_flash_lock_sector_t& sbp_msg, const uint8_t len = 0);
MsgStmFlashUnlockSector convertSbpMsgToRosMsg(const msg_stm_flash_unlock_sector_t& sbp_msg, const uint8_t len = 0);
MsgStmUniqueIdResp convertSbpMsgToRosMsg(const msg_stm_unique_id_resp_t& sbp_msg, const uint8_t len = 0);
MsgM25FlashWriteStatus convertSbpMsgToRosMsg(const msg_m25_flash_write_status_t& sbp_msg, const uint8_t len = 0);
GnssSignal convertSbpMsgToRosMsg(const sbp_gnss_signal_t& sbp_msg, const uint8_t len = 0);
SvId convertSbpMsgToRosMsg(const sv_id_t& sbp_msg, const uint8_t len = 0);
GpsTimeSec convertSbpMsgToRosMsg(const gps_time_sec_t& sbp_msg, const uint8_t len = 0);
GpsTime convertSbpMsgToRosMsg(const sbp_gps_time_t& sbp_msg, const uint8_t len = 0);
CarrierPhase convertSbpMsgToRosMsg(const carrier_phase_t& sbp_msg, const uint8_t len = 0);
MsgImuRaw convertSbpMsgToRosMsg(const msg_imu_raw_t& sbp_msg, const uint8_t len = 0);
MsgImuAux convertSbpMsgToRosMsg(const msg_imu_aux_t& sbp_msg, const uint8_t len = 0);
MsgLinuxCpuState convertSbpMsgToRosMsg(const msg_linux_cpu_state_t& sbp_msg, const uint8_t len = 0);
MsgLinuxMemState convertSbpMsgToRosMsg(const msg_linux_mem_state_t& sbp_msg, const uint8_t len = 0);
MsgLinuxSysState convertSbpMsgToRosMsg(const msg_linux_sys_state_t& sbp_msg, const uint8_t len = 0);
MsgLinuxProcessSocketCounts convertSbpMsgToRosMsg(const msg_linux_process_socket_counts_t& sbp_msg, const uint8_t len = 0);
MsgLinuxProcessSocketQueues convertSbpMsgToRosMsg(const msg_linux_process_socket_queues_t& sbp_msg, const uint8_t len = 0);
MsgLinuxSocketUsage convertSbpMsgToRosMsg(const msg_linux_socket_usage_t& sbp_msg, const uint8_t len = 0);
MsgLinuxProcessFdCount convertSbpMsgToRosMsg(const msg_linux_process_fd_count_t& sbp_msg, const uint8_t len = 0);
MsgLinuxProcessFdSummary convertSbpMsgToRosMsg(const msg_linux_process_fd_summary_t& sbp_msg, const uint8_t len = 0);
MsgLog convertSbpMsgToRosMsg(const msg_log_t& sbp_msg, const uint8_t len = 0);
MsgFwd convertSbpMsgToRosMsg(const msg_fwd_t& sbp_msg, const uint8_t len = 0);
MsgMagRaw convertSbpMsgToRosMsg(const msg_mag_raw_t& sbp_msg, const uint8_t len = 0);
MsgGpsTime convertSbpMsgToRosMsg(const msg_gps_time_t& sbp_msg, const uint8_t len = 0);
MsgUtcTime convertSbpMsgToRosMsg(const msg_utc_time_t& sbp_msg, const uint8_t len = 0);
MsgDops convertSbpMsgToRosMsg(const msg_dops_t& sbp_msg, const uint8_t len = 0);
MsgPosEcef convertSbpMsgToRosMsg(const msg_pos_ecef_t& sbp_msg, const uint8_t len = 0);
MsgPosEcefCov convertSbpMsgToRosMsg(const msg_pos_ecef_cov_t& sbp_msg, const uint8_t len = 0);
MsgPosLlh convertSbpMsgToRosMsg(const msg_pos_llh_t& sbp_msg, const uint8_t len = 0);
MsgPosLlhCov convertSbpMsgToRosMsg(const msg_pos_llh_cov_t& sbp_msg, const uint8_t len = 0);
MsgBaselineEcef convertSbpMsgToRosMsg(const msg_baseline_ecef_t& sbp_msg, const uint8_t len = 0);
MsgBaselineNed convertSbpMsgToRosMsg(const msg_baseline_ned_t& sbp_msg, const uint8_t len = 0);
MsgVelEcef convertSbpMsgToRosMsg(const msg_vel_ecef_t& sbp_msg, const uint8_t len = 0);
MsgVelEcefCov convertSbpMsgToRosMsg(const msg_vel_ecef_cov_t& sbp_msg, const uint8_t len = 0);
MsgVelNed convertSbpMsgToRosMsg(const msg_vel_ned_t& sbp_msg, const uint8_t len = 0);
MsgVelNedCov convertSbpMsgToRosMsg(const msg_vel_ned_cov_t& sbp_msg, const uint8_t len = 0);
MsgVelBody convertSbpMsgToRosMsg(const msg_vel_body_t& sbp_msg, const uint8_t len = 0);
MsgAgeCorrections convertSbpMsgToRosMsg(const msg_age_corrections_t& sbp_msg, const uint8_t len = 0);
MsgNdbEvent convertSbpMsgToRosMsg(const msg_ndb_event_t& sbp_msg, const uint8_t len = 0);
ObservationHeader convertSbpMsgToRosMsg(const observation_header_t& sbp_msg, const uint8_t len = 0);
Doppler convertSbpMsgToRosMsg(const doppler_t& sbp_msg, const uint8_t len = 0);
PackedObsContent convertSbpMsgToRosMsg(const packed_obs_content_t& sbp_msg, const uint8_t len = 0);
PackedOsrContent convertSbpMsgToRosMsg(const packed_osr_content_t& sbp_msg, const uint8_t len = 0);
MsgObs convertSbpMsgToRosMsg(const msg_obs_t& sbp_msg, const uint8_t len = 0);
MsgBasePosLlh convertSbpMsgToRosMsg(const msg_base_pos_llh_t& sbp_msg, const uint8_t len = 0);
MsgBasePosEcef convertSbpMsgToRosMsg(const msg_base_pos_ecef_t& sbp_msg, const uint8_t len = 0);
EphemerisCommonContent convertSbpMsgToRosMsg(const ephemeris_common_content_t& sbp_msg, const uint8_t len = 0);
MsgEphemerisGps convertSbpMsgToRosMsg(const msg_ephemeris_gps_t& sbp_msg, const uint8_t len = 0);
MsgEphemerisQzss convertSbpMsgToRosMsg(const msg_ephemeris_qzss_t& sbp_msg, const uint8_t len = 0);
MsgEphemerisBds convertSbpMsgToRosMsg(const msg_ephemeris_bds_t& sbp_msg, const uint8_t len = 0);
MsgEphemerisGal convertSbpMsgToRosMsg(const msg_ephemeris_gal_t& sbp_msg, const uint8_t len = 0);
MsgEphemerisSbas convertSbpMsgToRosMsg(const msg_ephemeris_sbas_t& sbp_msg, const uint8_t len = 0);
MsgEphemerisGlo convertSbpMsgToRosMsg(const msg_ephemeris_glo_t& sbp_msg, const uint8_t len = 0);
MsgIono convertSbpMsgToRosMsg(const msg_iono_t& sbp_msg, const uint8_t len = 0);
GnssCapb convertSbpMsgToRosMsg(const gnss_capb_t& sbp_msg, const uint8_t len = 0);
MsgGnssCapb convertSbpMsgToRosMsg(const msg_gnss_capb_t& sbp_msg, const uint8_t len = 0);
MsgGroupDelay convertSbpMsgToRosMsg(const msg_group_delay_t& sbp_msg, const uint8_t len = 0);
AlmanacCommonContent convertSbpMsgToRosMsg(const almanac_common_content_t& sbp_msg, const uint8_t len = 0);
MsgAlmanacGps convertSbpMsgToRosMsg(const msg_almanac_gps_t& sbp_msg, const uint8_t len = 0);
MsgAlmanacGlo convertSbpMsgToRosMsg(const msg_almanac_glo_t& sbp_msg, const uint8_t len = 0);
MsgGloBiases convertSbpMsgToRosMsg(const msg_glo_biases_t& sbp_msg, const uint8_t len = 0);
SvAzEl convertSbpMsgToRosMsg(const sv_az_el_t& sbp_msg, const uint8_t len = 0);
MsgSvAzEl convertSbpMsgToRosMsg(const msg_sv_az_el_t& sbp_msg, const uint8_t len = 0);
MsgOsr convertSbpMsgToRosMsg(const msg_osr_t& sbp_msg, const uint8_t len = 0);
MsgBaselineHeading convertSbpMsgToRosMsg(const msg_baseline_heading_t& sbp_msg, const uint8_t len = 0);
MsgOrientQuat convertSbpMsgToRosMsg(const msg_orient_quat_t& sbp_msg, const uint8_t len = 0);
MsgOrientEuler convertSbpMsgToRosMsg(const msg_orient_euler_t& sbp_msg, const uint8_t len = 0);
MsgAngularRate convertSbpMsgToRosMsg(const msg_angular_rate_t& sbp_msg, const uint8_t len = 0);
MsgReset convertSbpMsgToRosMsg(const msg_reset_t& sbp_msg, const uint8_t len = 0);
MsgResetFilters convertSbpMsgToRosMsg(const msg_reset_filters_t& sbp_msg, const uint8_t len = 0);
MsgThreadState convertSbpMsgToRosMsg(const msg_thread_state_t& sbp_msg, const uint8_t len = 0);
UartChannel convertSbpMsgToRosMsg(const uart_channel_t& sbp_msg, const uint8_t len = 0);
Period convertSbpMsgToRosMsg(const period_t& sbp_msg, const uint8_t len = 0);
Latency convertSbpMsgToRosMsg(const latency_t& sbp_msg, const uint8_t len = 0);
MsgUartState convertSbpMsgToRosMsg(const msg_uart_state_t& sbp_msg, const uint8_t len = 0);
MsgIarState convertSbpMsgToRosMsg(const msg_iar_state_t& sbp_msg, const uint8_t len = 0);
MsgMaskSatellite convertSbpMsgToRosMsg(const msg_mask_satellite_t& sbp_msg, const uint8_t len = 0);
MsgDeviceMonitor convertSbpMsgToRosMsg(const msg_device_monitor_t& sbp_msg, const uint8_t len = 0);
MsgCommandReq convertSbpMsgToRosMsg(const msg_command_req_t& sbp_msg, const uint8_t len = 0);
MsgCommandResp convertSbpMsgToRosMsg(const msg_command_resp_t& sbp_msg, const uint8_t len = 0);
MsgCommandOutput convertSbpMsgToRosMsg(const msg_command_output_t& sbp_msg, const uint8_t len = 0);
MsgNetworkStateResp convertSbpMsgToRosMsg(const msg_network_state_resp_t& sbp_msg, const uint8_t len = 0);
NetworkUsage convertSbpMsgToRosMsg(const network_usage_t& sbp_msg, const uint8_t len = 0);
MsgNetworkBandwidthUsage convertSbpMsgToRosMsg(const msg_network_bandwidth_usage_t& sbp_msg, const uint8_t len = 0);
MsgCellModemStatus convertSbpMsgToRosMsg(const msg_cell_modem_status_t& sbp_msg, const uint8_t len = 0);
MsgSpecan convertSbpMsgToRosMsg(const msg_specan_t& sbp_msg, const uint8_t len = 0);
MsgFrontEndGain convertSbpMsgToRosMsg(const msg_front_end_gain_t& sbp_msg, const uint8_t len = 0);
MsgSbasRaw convertSbpMsgToRosMsg(const msg_sbas_raw_t& sbp_msg, const uint8_t len = 0);
MsgSettingsWrite convertSbpMsgToRosMsg(const msg_settings_write_t& sbp_msg, const uint8_t len = 0);
MsgSettingsWriteResp convertSbpMsgToRosMsg(const msg_settings_write_resp_t& sbp_msg, const uint8_t len = 0);
MsgSettingsReadReq convertSbpMsgToRosMsg(const msg_settings_read_req_t& sbp_msg, const uint8_t len = 0);
MsgSettingsReadResp convertSbpMsgToRosMsg(const msg_settings_read_resp_t& sbp_msg, const uint8_t len = 0);
MsgSettingsReadByIndexReq convertSbpMsgToRosMsg(const msg_settings_read_by_index_req_t& sbp_msg, const uint8_t len = 0);
MsgSettingsReadByIndexResp convertSbpMsgToRosMsg(const msg_settings_read_by_index_resp_t& sbp_msg, const uint8_t len = 0);
MsgSettingsRegister convertSbpMsgToRosMsg(const msg_settings_register_t& sbp_msg, const uint8_t len = 0);
MsgSettingsRegisterResp convertSbpMsgToRosMsg(const msg_settings_register_resp_t& sbp_msg, const uint8_t len = 0);
CodeBiasesContent convertSbpMsgToRosMsg(const code_biases_content_t& sbp_msg, const uint8_t len = 0);
PhaseBiasesContent convertSbpMsgToRosMsg(const phase_biases_content_t& sbp_msg, const uint8_t len = 0);
StecHeader convertSbpMsgToRosMsg(const stec_header_t& sbp_msg, const uint8_t len = 0);
GriddedCorrectionHeader convertSbpMsgToRosMsg(const gridded_correction_header_t& sbp_msg, const uint8_t len = 0);
StecSatElement convertSbpMsgToRosMsg(const stec_sat_element_t& sbp_msg, const uint8_t len = 0);
TroposphericDelayCorrection convertSbpMsgToRosMsg(const tropospheric_delay_correction_t& sbp_msg, const uint8_t len = 0);
StecResidual convertSbpMsgToRosMsg(const stec_residual_t& sbp_msg, const uint8_t len = 0);
GridElement convertSbpMsgToRosMsg(const grid_element_t& sbp_msg, const uint8_t len = 0);
GridDefinitionHeader convertSbpMsgToRosMsg(const grid_definition_header_t& sbp_msg, const uint8_t len = 0);
MsgSsrOrbitClock convertSbpMsgToRosMsg(const msg_ssr_orbit_clock_t& sbp_msg, const uint8_t len = 0);
MsgSsrCodeBiases convertSbpMsgToRosMsg(const msg_ssr_code_biases_t& sbp_msg, const uint8_t len = 0);
MsgSsrPhaseBiases convertSbpMsgToRosMsg(const msg_ssr_phase_biases_t& sbp_msg, const uint8_t len = 0);
MsgSsrStecCorrection convertSbpMsgToRosMsg(const msg_ssr_stec_correction_t& sbp_msg, const uint8_t len = 0);
MsgSsrGriddedCorrection convertSbpMsgToRosMsg(const msg_ssr_gridded_correction_t& sbp_msg, const uint8_t len = 0);
MsgSsrGridDefinition convertSbpMsgToRosMsg(const msg_ssr_grid_definition_t& sbp_msg, const uint8_t len = 0);
MsgStartup convertSbpMsgToRosMsg(const msg_startup_t& sbp_msg, const uint8_t len = 0);
MsgDgnssStatus convertSbpMsgToRosMsg(const msg_dgnss_status_t& sbp_msg, const uint8_t len = 0);
MsgHeartbeat convertSbpMsgToRosMsg(const msg_heartbeat_t& sbp_msg, const uint8_t len = 0);
MsgInsStatus convertSbpMsgToRosMsg(const msg_ins_status_t& sbp_msg, const uint8_t len = 0);
MsgCsacTelemetry convertSbpMsgToRosMsg(const msg_csac_telemetry_t& sbp_msg, const uint8_t len = 0);
MsgCsacTelemetryLabels convertSbpMsgToRosMsg(const msg_csac_telemetry_labels_t& sbp_msg, const uint8_t len = 0);
TrackingChannelState convertSbpMsgToRosMsg(const tracking_channel_state_t& sbp_msg, const uint8_t len = 0);
MsgTrackingState convertSbpMsgToRosMsg(const msg_tracking_state_t& sbp_msg, const uint8_t len = 0);
MeasurementState convertSbpMsgToRosMsg(const measurement_state_t& sbp_msg, const uint8_t len = 0);
MsgMeasurementState convertSbpMsgToRosMsg(const msg_measurement_state_t& sbp_msg, const uint8_t len = 0);
TrackingChannelCorrelation convertSbpMsgToRosMsg(const tracking_channel_correlation_t& sbp_msg, const uint8_t len = 0);
MsgTrackingIq convertSbpMsgToRosMsg(const msg_tracking_iq_t& sbp_msg, const uint8_t len = 0);
MsgUserData convertSbpMsgToRosMsg(const msg_user_data_t& sbp_msg, const uint8_t len = 0);
MsgOdometry convertSbpMsgToRosMsg(const msg_odometry_t& sbp_msg, const uint8_t len = 0);
} // namespace piksi_multi_msgs

#endif // PIKSI_MULTI_MSGS_CONVERSION_H_