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

#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBP_CALLBACK_HANDLER_RELAY_SBP_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBP_CALLBACK_HANDLER_RELAY_SBP_H_

#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"
#include <piksi_multi_msgs/conversion.h>

namespace piksi_multi_cpp {

// Declare all relays.

class SBPCallbackHandlerRelayMsgAcqResult
    : public SBPCallbackHandlerRelay<msg_acq_result_t,
                                     piksi_multi_msgs::MsgAcqResult> {
  public:
    inline SBPCallbackHandlerRelayMsgAcqResult(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_ACQ_RESULT, state, "acq_result") {}
};

class SBPCallbackHandlerRelayMsgAcqSvProfile
    : public SBPCallbackHandlerRelay<msg_acq_sv_profile_t,
                                     piksi_multi_msgs::MsgAcqSvProfile> {
  public:
    inline SBPCallbackHandlerRelayMsgAcqSvProfile(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_ACQ_SV_PROFILE, state, "acq_sv_profile") {}
};

class SBPCallbackHandlerRelayMsgBootloaderHandshakeResp
    : public SBPCallbackHandlerRelay<msg_bootloader_handshake_resp_t,
                                     piksi_multi_msgs::MsgBootloaderHandshakeResp> {
  public:
    inline SBPCallbackHandlerRelayMsgBootloaderHandshakeResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_BOOTLOADER_HANDSHAKE_RESP, state, "bootloader_handshake_resp") {}
};

class SBPCallbackHandlerRelayMsgBootloaderJumpToApp
    : public SBPCallbackHandlerRelay<msg_bootloader_jump_to_app_t,
                                     piksi_multi_msgs::MsgBootloaderJumpToApp> {
  public:
    inline SBPCallbackHandlerRelayMsgBootloaderJumpToApp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_BOOTLOADER_JUMP_TO_APP, state, "bootloader_jump_to_app") {}
};

class SBPCallbackHandlerRelayMsgNapDeviceDnaResp
    : public SBPCallbackHandlerRelay<msg_nap_device_dna_resp_t,
                                     piksi_multi_msgs::MsgNapDeviceDnaResp> {
  public:
    inline SBPCallbackHandlerRelayMsgNapDeviceDnaResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_NAP_DEVICE_DNA_RESP, state, "nap_device_dna_resp") {}
};

class SBPCallbackHandlerRelayMsgExtEvent
    : public SBPCallbackHandlerRelay<msg_ext_event_t,
                                     piksi_multi_msgs::MsgExtEvent> {
  public:
    inline SBPCallbackHandlerRelayMsgExtEvent(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_EXT_EVENT, state, "ext_event") {}
};

class SBPCallbackHandlerRelayMsgFileioReadReq
    : public SBPCallbackHandlerRelay<msg_fileio_read_req_t,
                                     piksi_multi_msgs::MsgFileioReadReq> {
  public:
    inline SBPCallbackHandlerRelayMsgFileioReadReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FILEIO_READ_REQ, state, "fileio_read_req") {}
};

class SBPCallbackHandlerRelayMsgFileioReadResp
    : public SBPCallbackHandlerRelay<msg_fileio_read_resp_t,
                                     piksi_multi_msgs::MsgFileioReadResp> {
  public:
    inline SBPCallbackHandlerRelayMsgFileioReadResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FILEIO_READ_RESP, state, "fileio_read_resp") {}
};

class SBPCallbackHandlerRelayMsgFileioReadDirReq
    : public SBPCallbackHandlerRelay<msg_fileio_read_dir_req_t,
                                     piksi_multi_msgs::MsgFileioReadDirReq> {
  public:
    inline SBPCallbackHandlerRelayMsgFileioReadDirReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FILEIO_READ_DIR_REQ, state, "fileio_read_dir_req") {}
};

class SBPCallbackHandlerRelayMsgFileioReadDirResp
    : public SBPCallbackHandlerRelay<msg_fileio_read_dir_resp_t,
                                     piksi_multi_msgs::MsgFileioReadDirResp> {
  public:
    inline SBPCallbackHandlerRelayMsgFileioReadDirResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FILEIO_READ_DIR_RESP, state, "fileio_read_dir_resp") {}
};

class SBPCallbackHandlerRelayMsgFileioRemove
    : public SBPCallbackHandlerRelay<msg_fileio_remove_t,
                                     piksi_multi_msgs::MsgFileioRemove> {
  public:
    inline SBPCallbackHandlerRelayMsgFileioRemove(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FILEIO_REMOVE, state, "fileio_remove") {}
};

class SBPCallbackHandlerRelayMsgFileioWriteReq
    : public SBPCallbackHandlerRelay<msg_fileio_write_req_t,
                                     piksi_multi_msgs::MsgFileioWriteReq> {
  public:
    inline SBPCallbackHandlerRelayMsgFileioWriteReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FILEIO_WRITE_REQ, state, "fileio_write_req") {}
};

class SBPCallbackHandlerRelayMsgFileioWriteResp
    : public SBPCallbackHandlerRelay<msg_fileio_write_resp_t,
                                     piksi_multi_msgs::MsgFileioWriteResp> {
  public:
    inline SBPCallbackHandlerRelayMsgFileioWriteResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FILEIO_WRITE_RESP, state, "fileio_write_resp") {}
};

class SBPCallbackHandlerRelayMsgFileioConfigReq
    : public SBPCallbackHandlerRelay<msg_fileio_config_req_t,
                                     piksi_multi_msgs::MsgFileioConfigReq> {
  public:
    inline SBPCallbackHandlerRelayMsgFileioConfigReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FILEIO_CONFIG_REQ, state, "fileio_config_req") {}
};

class SBPCallbackHandlerRelayMsgFileioConfigResp
    : public SBPCallbackHandlerRelay<msg_fileio_config_resp_t,
                                     piksi_multi_msgs::MsgFileioConfigResp> {
  public:
    inline SBPCallbackHandlerRelayMsgFileioConfigResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FILEIO_CONFIG_RESP, state, "fileio_config_resp") {}
};

class SBPCallbackHandlerRelayMsgFlashProgram
    : public SBPCallbackHandlerRelay<msg_flash_program_t,
                                     piksi_multi_msgs::MsgFlashProgram> {
  public:
    inline SBPCallbackHandlerRelayMsgFlashProgram(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FLASH_PROGRAM, state, "flash_program") {}
};

class SBPCallbackHandlerRelayMsgFlashDone
    : public SBPCallbackHandlerRelay<msg_flash_done_t,
                                     piksi_multi_msgs::MsgFlashDone> {
  public:
    inline SBPCallbackHandlerRelayMsgFlashDone(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FLASH_DONE, state, "flash_done") {}
};

class SBPCallbackHandlerRelayMsgFlashReadReq
    : public SBPCallbackHandlerRelay<msg_flash_read_req_t,
                                     piksi_multi_msgs::MsgFlashReadReq> {
  public:
    inline SBPCallbackHandlerRelayMsgFlashReadReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FLASH_READ_REQ, state, "flash_read_req") {}
};

class SBPCallbackHandlerRelayMsgFlashReadResp
    : public SBPCallbackHandlerRelay<msg_flash_read_resp_t,
                                     piksi_multi_msgs::MsgFlashReadResp> {
  public:
    inline SBPCallbackHandlerRelayMsgFlashReadResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FLASH_READ_RESP, state, "flash_read_resp") {}
};

class SBPCallbackHandlerRelayMsgFlashErase
    : public SBPCallbackHandlerRelay<msg_flash_erase_t,
                                     piksi_multi_msgs::MsgFlashErase> {
  public:
    inline SBPCallbackHandlerRelayMsgFlashErase(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FLASH_ERASE, state, "flash_erase") {}
};

class SBPCallbackHandlerRelayMsgStmFlashLockSector
    : public SBPCallbackHandlerRelay<msg_stm_flash_lock_sector_t,
                                     piksi_multi_msgs::MsgStmFlashLockSector> {
  public:
    inline SBPCallbackHandlerRelayMsgStmFlashLockSector(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_STM_FLASH_LOCK_SECTOR, state, "stm_flash_lock_sector") {}
};

class SBPCallbackHandlerRelayMsgStmFlashUnlockSector
    : public SBPCallbackHandlerRelay<msg_stm_flash_unlock_sector_t,
                                     piksi_multi_msgs::MsgStmFlashUnlockSector> {
  public:
    inline SBPCallbackHandlerRelayMsgStmFlashUnlockSector(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_STM_FLASH_UNLOCK_SECTOR, state, "stm_flash_unlock_sector") {}
};

class SBPCallbackHandlerRelayMsgStmUniqueIdResp
    : public SBPCallbackHandlerRelay<msg_stm_unique_id_resp_t,
                                     piksi_multi_msgs::MsgStmUniqueIdResp> {
  public:
    inline SBPCallbackHandlerRelayMsgStmUniqueIdResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_STM_UNIQUE_ID_RESP, state, "stm_unique_id_resp") {}
};

class SBPCallbackHandlerRelayMsgM25FlashWriteStatus
    : public SBPCallbackHandlerRelay<msg_m25_flash_write_status_t,
                                     piksi_multi_msgs::MsgM25FlashWriteStatus> {
  public:
    inline SBPCallbackHandlerRelayMsgM25FlashWriteStatus(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_M25_FLASH_WRITE_STATUS, state, "m25_flash_write_status") {}
};

class SBPCallbackHandlerRelayMsgImuRaw
    : public SBPCallbackHandlerRelay<msg_imu_raw_t,
                                     piksi_multi_msgs::MsgImuRaw> {
  public:
    inline SBPCallbackHandlerRelayMsgImuRaw(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_IMU_RAW, state, "imu_raw") {}
};

class SBPCallbackHandlerRelayMsgImuAux
    : public SBPCallbackHandlerRelay<msg_imu_aux_t,
                                     piksi_multi_msgs::MsgImuAux> {
  public:
    inline SBPCallbackHandlerRelayMsgImuAux(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_IMU_AUX, state, "imu_aux") {}
};

class SBPCallbackHandlerRelayMsgLinuxCpuState
    : public SBPCallbackHandlerRelay<msg_linux_cpu_state_t,
                                     piksi_multi_msgs::MsgLinuxCpuState> {
  public:
    inline SBPCallbackHandlerRelayMsgLinuxCpuState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_CPU_STATE, state, "linux_cpu_state") {}
};

class SBPCallbackHandlerRelayMsgLinuxMemState
    : public SBPCallbackHandlerRelay<msg_linux_mem_state_t,
                                     piksi_multi_msgs::MsgLinuxMemState> {
  public:
    inline SBPCallbackHandlerRelayMsgLinuxMemState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_MEM_STATE, state, "linux_mem_state") {}
};

class SBPCallbackHandlerRelayMsgLinuxSysState
    : public SBPCallbackHandlerRelay<msg_linux_sys_state_t,
                                     piksi_multi_msgs::MsgLinuxSysState> {
  public:
    inline SBPCallbackHandlerRelayMsgLinuxSysState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_SYS_STATE, state, "linux_sys_state") {}
};

class SBPCallbackHandlerRelayMsgLinuxProcessSocketCounts
    : public SBPCallbackHandlerRelay<msg_linux_process_socket_counts_t,
                                     piksi_multi_msgs::MsgLinuxProcessSocketCounts> {
  public:
    inline SBPCallbackHandlerRelayMsgLinuxProcessSocketCounts(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_PROCESS_SOCKET_COUNTS, state, "linux_process_socket_counts") {}
};

class SBPCallbackHandlerRelayMsgLinuxProcessSocketQueues
    : public SBPCallbackHandlerRelay<msg_linux_process_socket_queues_t,
                                     piksi_multi_msgs::MsgLinuxProcessSocketQueues> {
  public:
    inline SBPCallbackHandlerRelayMsgLinuxProcessSocketQueues(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_PROCESS_SOCKET_QUEUES, state, "linux_process_socket_queues") {}
};

class SBPCallbackHandlerRelayMsgLinuxSocketUsage
    : public SBPCallbackHandlerRelay<msg_linux_socket_usage_t,
                                     piksi_multi_msgs::MsgLinuxSocketUsage> {
  public:
    inline SBPCallbackHandlerRelayMsgLinuxSocketUsage(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_SOCKET_USAGE, state, "linux_socket_usage") {}
};

class SBPCallbackHandlerRelayMsgLinuxProcessFdCount
    : public SBPCallbackHandlerRelay<msg_linux_process_fd_count_t,
                                     piksi_multi_msgs::MsgLinuxProcessFdCount> {
  public:
    inline SBPCallbackHandlerRelayMsgLinuxProcessFdCount(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_PROCESS_FD_COUNT, state, "linux_process_fd_count") {}
};

class SBPCallbackHandlerRelayMsgLinuxProcessFdSummary
    : public SBPCallbackHandlerRelay<msg_linux_process_fd_summary_t,
                                     piksi_multi_msgs::MsgLinuxProcessFdSummary> {
  public:
    inline SBPCallbackHandlerRelayMsgLinuxProcessFdSummary(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_LINUX_PROCESS_FD_SUMMARY, state, "linux_process_fd_summary") {}
};

class SBPCallbackHandlerRelayMsgLog
    : public SBPCallbackHandlerRelay<msg_log_t,
                                     piksi_multi_msgs::MsgLog> {
  public:
    inline SBPCallbackHandlerRelayMsgLog(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_LOG, state, "log") {}
};

class SBPCallbackHandlerRelayMsgFwd
    : public SBPCallbackHandlerRelay<msg_fwd_t,
                                     piksi_multi_msgs::MsgFwd> {
  public:
    inline SBPCallbackHandlerRelayMsgFwd(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FWD, state, "fwd") {}
};

class SBPCallbackHandlerRelayMsgMagRaw
    : public SBPCallbackHandlerRelay<msg_mag_raw_t,
                                     piksi_multi_msgs::MsgMagRaw> {
  public:
    inline SBPCallbackHandlerRelayMsgMagRaw(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_MAG_RAW, state, "mag_raw") {}
};

class SBPCallbackHandlerRelayMsgGpsTime
    : public SBPCallbackHandlerRelay<msg_gps_time_t,
                                     piksi_multi_msgs::MsgGpsTime> {
  public:
    inline SBPCallbackHandlerRelayMsgGpsTime(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_GPS_TIME, state, "gps_time") {}
};

class SBPCallbackHandlerRelayMsgUtcTime
    : public SBPCallbackHandlerRelay<msg_utc_time_t,
                                     piksi_multi_msgs::MsgUtcTime> {
  public:
    inline SBPCallbackHandlerRelayMsgUtcTime(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_UTC_TIME, state, "utc_time") {}
};

class SBPCallbackHandlerRelayMsgDops
    : public SBPCallbackHandlerRelay<msg_dops_t,
                                     piksi_multi_msgs::MsgDops> {
  public:
    inline SBPCallbackHandlerRelayMsgDops(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_DOPS, state, "dops") {}
};

class SBPCallbackHandlerRelayMsgPosEcef
    : public SBPCallbackHandlerRelay<msg_pos_ecef_t,
                                     piksi_multi_msgs::MsgPosEcef> {
  public:
    inline SBPCallbackHandlerRelayMsgPosEcef(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_POS_ECEF, state, "pos_ecef") {}
};

class SBPCallbackHandlerRelayMsgPosEcefCov
    : public SBPCallbackHandlerRelay<msg_pos_ecef_cov_t,
                                     piksi_multi_msgs::MsgPosEcefCov> {
  public:
    inline SBPCallbackHandlerRelayMsgPosEcefCov(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_POS_ECEF_COV, state, "pos_ecef_cov") {}
};

class SBPCallbackHandlerRelayMsgPosLlh
    : public SBPCallbackHandlerRelay<msg_pos_llh_t,
                                     piksi_multi_msgs::MsgPosLlh> {
  public:
    inline SBPCallbackHandlerRelayMsgPosLlh(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_POS_LLH, state, "pos_llh") {}
};

class SBPCallbackHandlerRelayMsgPosLlhCov
    : public SBPCallbackHandlerRelay<msg_pos_llh_cov_t,
                                     piksi_multi_msgs::MsgPosLlhCov> {
  public:
    inline SBPCallbackHandlerRelayMsgPosLlhCov(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_POS_LLH_COV, state, "pos_llh_cov") {}
};

class SBPCallbackHandlerRelayMsgBaselineEcef
    : public SBPCallbackHandlerRelay<msg_baseline_ecef_t,
                                     piksi_multi_msgs::MsgBaselineEcef> {
  public:
    inline SBPCallbackHandlerRelayMsgBaselineEcef(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_BASELINE_ECEF, state, "baseline_ecef") {}
};

class SBPCallbackHandlerRelayMsgBaselineNed
    : public SBPCallbackHandlerRelay<msg_baseline_ned_t,
                                     piksi_multi_msgs::MsgBaselineNed> {
  public:
    inline SBPCallbackHandlerRelayMsgBaselineNed(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_BASELINE_NED, state, "baseline_ned") {}
};

class SBPCallbackHandlerRelayMsgVelEcef
    : public SBPCallbackHandlerRelay<msg_vel_ecef_t,
                                     piksi_multi_msgs::MsgVelEcef> {
  public:
    inline SBPCallbackHandlerRelayMsgVelEcef(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_ECEF, state, "vel_ecef") {}
};

class SBPCallbackHandlerRelayMsgVelEcefCov
    : public SBPCallbackHandlerRelay<msg_vel_ecef_cov_t,
                                     piksi_multi_msgs::MsgVelEcefCov> {
  public:
    inline SBPCallbackHandlerRelayMsgVelEcefCov(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_ECEF_COV, state, "vel_ecef_cov") {}
};

class SBPCallbackHandlerRelayMsgVelNed
    : public SBPCallbackHandlerRelay<msg_vel_ned_t,
                                     piksi_multi_msgs::MsgVelNed> {
  public:
    inline SBPCallbackHandlerRelayMsgVelNed(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_NED, state, "vel_ned") {}
};

class SBPCallbackHandlerRelayMsgVelNedCov
    : public SBPCallbackHandlerRelay<msg_vel_ned_cov_t,
                                     piksi_multi_msgs::MsgVelNedCov> {
  public:
    inline SBPCallbackHandlerRelayMsgVelNedCov(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_NED_COV, state, "vel_ned_cov") {}
};

class SBPCallbackHandlerRelayMsgVelBody
    : public SBPCallbackHandlerRelay<msg_vel_body_t,
                                     piksi_multi_msgs::MsgVelBody> {
  public:
    inline SBPCallbackHandlerRelayMsgVelBody(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_BODY, state, "vel_body") {}
};

class SBPCallbackHandlerRelayMsgAgeCorrections
    : public SBPCallbackHandlerRelay<msg_age_corrections_t,
                                     piksi_multi_msgs::MsgAgeCorrections> {
  public:
    inline SBPCallbackHandlerRelayMsgAgeCorrections(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_AGE_CORRECTIONS, state, "age_corrections") {}
};

class SBPCallbackHandlerRelayMsgNdbEvent
    : public SBPCallbackHandlerRelay<msg_ndb_event_t,
                                     piksi_multi_msgs::MsgNdbEvent> {
  public:
    inline SBPCallbackHandlerRelayMsgNdbEvent(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_NDB_EVENT, state, "ndb_event") {}
};

class SBPCallbackHandlerRelayMsgObs
    : public SBPCallbackHandlerRelay<msg_obs_t,
                                     piksi_multi_msgs::MsgObs> {
  public:
    inline SBPCallbackHandlerRelayMsgObs(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_OBS, state, "obs") {}
};

class SBPCallbackHandlerRelayMsgBasePosLlh
    : public SBPCallbackHandlerRelay<msg_base_pos_llh_t,
                                     piksi_multi_msgs::MsgBasePosLlh> {
  public:
    inline SBPCallbackHandlerRelayMsgBasePosLlh(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_BASE_POS_LLH, state, "base_pos_llh") {}
};

class SBPCallbackHandlerRelayMsgBasePosEcef
    : public SBPCallbackHandlerRelay<msg_base_pos_ecef_t,
                                     piksi_multi_msgs::MsgBasePosEcef> {
  public:
    inline SBPCallbackHandlerRelayMsgBasePosEcef(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_BASE_POS_ECEF, state, "base_pos_ecef") {}
};

class SBPCallbackHandlerRelayMsgEphemerisGps
    : public SBPCallbackHandlerRelay<msg_ephemeris_gps_t,
                                     piksi_multi_msgs::MsgEphemerisGps> {
  public:
    inline SBPCallbackHandlerRelayMsgEphemerisGps(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_EPHEMERIS_GPS, state, "ephemeris_gps") {}
};

class SBPCallbackHandlerRelayMsgEphemerisQzss
    : public SBPCallbackHandlerRelay<msg_ephemeris_qzss_t,
                                     piksi_multi_msgs::MsgEphemerisQzss> {
  public:
    inline SBPCallbackHandlerRelayMsgEphemerisQzss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_EPHEMERIS_QZSS, state, "ephemeris_qzss") {}
};

class SBPCallbackHandlerRelayMsgEphemerisBds
    : public SBPCallbackHandlerRelay<msg_ephemeris_bds_t,
                                     piksi_multi_msgs::MsgEphemerisBds> {
  public:
    inline SBPCallbackHandlerRelayMsgEphemerisBds(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_EPHEMERIS_BDS, state, "ephemeris_bds") {}
};

class SBPCallbackHandlerRelayMsgEphemerisGal
    : public SBPCallbackHandlerRelay<msg_ephemeris_gal_t,
                                     piksi_multi_msgs::MsgEphemerisGal> {
  public:
    inline SBPCallbackHandlerRelayMsgEphemerisGal(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_EPHEMERIS_GAL, state, "ephemeris_gal") {}
};

class SBPCallbackHandlerRelayMsgEphemerisSbas
    : public SBPCallbackHandlerRelay<msg_ephemeris_sbas_t,
                                     piksi_multi_msgs::MsgEphemerisSbas> {
  public:
    inline SBPCallbackHandlerRelayMsgEphemerisSbas(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_EPHEMERIS_SBAS, state, "ephemeris_sbas") {}
};

class SBPCallbackHandlerRelayMsgEphemerisGlo
    : public SBPCallbackHandlerRelay<msg_ephemeris_glo_t,
                                     piksi_multi_msgs::MsgEphemerisGlo> {
  public:
    inline SBPCallbackHandlerRelayMsgEphemerisGlo(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_EPHEMERIS_GLO, state, "ephemeris_glo") {}
};

class SBPCallbackHandlerRelayMsgIono
    : public SBPCallbackHandlerRelay<msg_iono_t,
                                     piksi_multi_msgs::MsgIono> {
  public:
    inline SBPCallbackHandlerRelayMsgIono(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_IONO, state, "iono") {}
};

class SBPCallbackHandlerRelayMsgGnssCapb
    : public SBPCallbackHandlerRelay<msg_gnss_capb_t,
                                     piksi_multi_msgs::MsgGnssCapb> {
  public:
    inline SBPCallbackHandlerRelayMsgGnssCapb(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_GNSS_CAPB, state, "gnss_capb") {}
};

class SBPCallbackHandlerRelayMsgGroupDelay
    : public SBPCallbackHandlerRelay<msg_group_delay_t,
                                     piksi_multi_msgs::MsgGroupDelay> {
  public:
    inline SBPCallbackHandlerRelayMsgGroupDelay(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_GROUP_DELAY, state, "group_delay") {}
};

class SBPCallbackHandlerRelayMsgAlmanacGps
    : public SBPCallbackHandlerRelay<msg_almanac_gps_t,
                                     piksi_multi_msgs::MsgAlmanacGps> {
  public:
    inline SBPCallbackHandlerRelayMsgAlmanacGps(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_ALMANAC_GPS, state, "almanac_gps") {}
};

class SBPCallbackHandlerRelayMsgAlmanacGlo
    : public SBPCallbackHandlerRelay<msg_almanac_glo_t,
                                     piksi_multi_msgs::MsgAlmanacGlo> {
  public:
    inline SBPCallbackHandlerRelayMsgAlmanacGlo(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_ALMANAC_GLO, state, "almanac_glo") {}
};

class SBPCallbackHandlerRelayMsgGloBiases
    : public SBPCallbackHandlerRelay<msg_glo_biases_t,
                                     piksi_multi_msgs::MsgGloBiases> {
  public:
    inline SBPCallbackHandlerRelayMsgGloBiases(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_GLO_BIASES, state, "glo_biases") {}
};

class SBPCallbackHandlerRelayMsgSvAzEl
    : public SBPCallbackHandlerRelay<msg_sv_az_el_t,
                                     piksi_multi_msgs::MsgSvAzEl> {
  public:
    inline SBPCallbackHandlerRelayMsgSvAzEl(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SV_AZ_EL, state, "sv_az_el") {}
};

class SBPCallbackHandlerRelayMsgOsr
    : public SBPCallbackHandlerRelay<msg_osr_t,
                                     piksi_multi_msgs::MsgOsr> {
  public:
    inline SBPCallbackHandlerRelayMsgOsr(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_OSR, state, "osr") {}
};

class SBPCallbackHandlerRelayMsgBaselineHeading
    : public SBPCallbackHandlerRelay<msg_baseline_heading_t,
                                     piksi_multi_msgs::MsgBaselineHeading> {
  public:
    inline SBPCallbackHandlerRelayMsgBaselineHeading(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_BASELINE_HEADING, state, "baseline_heading") {}
};

class SBPCallbackHandlerRelayMsgOrientQuat
    : public SBPCallbackHandlerRelay<msg_orient_quat_t,
                                     piksi_multi_msgs::MsgOrientQuat> {
  public:
    inline SBPCallbackHandlerRelayMsgOrientQuat(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_ORIENT_QUAT, state, "orient_quat") {}
};

class SBPCallbackHandlerRelayMsgOrientEuler
    : public SBPCallbackHandlerRelay<msg_orient_euler_t,
                                     piksi_multi_msgs::MsgOrientEuler> {
  public:
    inline SBPCallbackHandlerRelayMsgOrientEuler(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_ORIENT_EULER, state, "orient_euler") {}
};

class SBPCallbackHandlerRelayMsgAngularRate
    : public SBPCallbackHandlerRelay<msg_angular_rate_t,
                                     piksi_multi_msgs::MsgAngularRate> {
  public:
    inline SBPCallbackHandlerRelayMsgAngularRate(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_ANGULAR_RATE, state, "angular_rate") {}
};

class SBPCallbackHandlerRelayMsgReset
    : public SBPCallbackHandlerRelay<msg_reset_t,
                                     piksi_multi_msgs::MsgReset> {
  public:
    inline SBPCallbackHandlerRelayMsgReset(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_RESET, state, "reset") {}
};

class SBPCallbackHandlerRelayMsgResetFilters
    : public SBPCallbackHandlerRelay<msg_reset_filters_t,
                                     piksi_multi_msgs::MsgResetFilters> {
  public:
    inline SBPCallbackHandlerRelayMsgResetFilters(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_RESET_FILTERS, state, "reset_filters") {}
};

class SBPCallbackHandlerRelayMsgThreadState
    : public SBPCallbackHandlerRelay<msg_thread_state_t,
                                     piksi_multi_msgs::MsgThreadState> {
  public:
    inline SBPCallbackHandlerRelayMsgThreadState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_THREAD_STATE, state, "thread_state") {}
};

class SBPCallbackHandlerRelayMsgUartState
    : public SBPCallbackHandlerRelay<msg_uart_state_t,
                                     piksi_multi_msgs::MsgUartState> {
  public:
    inline SBPCallbackHandlerRelayMsgUartState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_UART_STATE, state, "uart_state") {}
};

class SBPCallbackHandlerRelayMsgIarState
    : public SBPCallbackHandlerRelay<msg_iar_state_t,
                                     piksi_multi_msgs::MsgIarState> {
  public:
    inline SBPCallbackHandlerRelayMsgIarState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_IAR_STATE, state, "iar_state") {}
};

class SBPCallbackHandlerRelayMsgMaskSatellite
    : public SBPCallbackHandlerRelay<msg_mask_satellite_t,
                                     piksi_multi_msgs::MsgMaskSatellite> {
  public:
    inline SBPCallbackHandlerRelayMsgMaskSatellite(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_MASK_SATELLITE, state, "mask_satellite") {}
};

class SBPCallbackHandlerRelayMsgDeviceMonitor
    : public SBPCallbackHandlerRelay<msg_device_monitor_t,
                                     piksi_multi_msgs::MsgDeviceMonitor> {
  public:
    inline SBPCallbackHandlerRelayMsgDeviceMonitor(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_DEVICE_MONITOR, state, "device_monitor") {}
};

class SBPCallbackHandlerRelayMsgCommandReq
    : public SBPCallbackHandlerRelay<msg_command_req_t,
                                     piksi_multi_msgs::MsgCommandReq> {
  public:
    inline SBPCallbackHandlerRelayMsgCommandReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_COMMAND_REQ, state, "command_req") {}
};

class SBPCallbackHandlerRelayMsgCommandResp
    : public SBPCallbackHandlerRelay<msg_command_resp_t,
                                     piksi_multi_msgs::MsgCommandResp> {
  public:
    inline SBPCallbackHandlerRelayMsgCommandResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_COMMAND_RESP, state, "command_resp") {}
};

class SBPCallbackHandlerRelayMsgCommandOutput
    : public SBPCallbackHandlerRelay<msg_command_output_t,
                                     piksi_multi_msgs::MsgCommandOutput> {
  public:
    inline SBPCallbackHandlerRelayMsgCommandOutput(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_COMMAND_OUTPUT, state, "command_output") {}
};

class SBPCallbackHandlerRelayMsgNetworkStateResp
    : public SBPCallbackHandlerRelay<msg_network_state_resp_t,
                                     piksi_multi_msgs::MsgNetworkStateResp> {
  public:
    inline SBPCallbackHandlerRelayMsgNetworkStateResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_NETWORK_STATE_RESP, state, "network_state_resp") {}
};

class SBPCallbackHandlerRelayMsgNetworkBandwidthUsage
    : public SBPCallbackHandlerRelay<msg_network_bandwidth_usage_t,
                                     piksi_multi_msgs::MsgNetworkBandwidthUsage> {
  public:
    inline SBPCallbackHandlerRelayMsgNetworkBandwidthUsage(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_NETWORK_BANDWIDTH_USAGE, state, "network_bandwidth_usage") {}
};

class SBPCallbackHandlerRelayMsgCellModemStatus
    : public SBPCallbackHandlerRelay<msg_cell_modem_status_t,
                                     piksi_multi_msgs::MsgCellModemStatus> {
  public:
    inline SBPCallbackHandlerRelayMsgCellModemStatus(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_CELL_MODEM_STATUS, state, "cell_modem_status") {}
};

class SBPCallbackHandlerRelayMsgSpecan
    : public SBPCallbackHandlerRelay<msg_specan_t,
                                     piksi_multi_msgs::MsgSpecan> {
  public:
    inline SBPCallbackHandlerRelayMsgSpecan(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SPECAN, state, "specan") {}
};

class SBPCallbackHandlerRelayMsgFrontEndGain
    : public SBPCallbackHandlerRelay<msg_front_end_gain_t,
                                     piksi_multi_msgs::MsgFrontEndGain> {
  public:
    inline SBPCallbackHandlerRelayMsgFrontEndGain(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_FRONT_END_GAIN, state, "front_end_gain") {}
};

class SBPCallbackHandlerRelayMsgSbasRaw
    : public SBPCallbackHandlerRelay<msg_sbas_raw_t,
                                     piksi_multi_msgs::MsgSbasRaw> {
  public:
    inline SBPCallbackHandlerRelayMsgSbasRaw(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SBAS_RAW, state, "sbas_raw") {}
};

class SBPCallbackHandlerRelayMsgSettingsWrite
    : public SBPCallbackHandlerRelay<msg_settings_write_t,
                                     piksi_multi_msgs::MsgSettingsWrite> {
  public:
    inline SBPCallbackHandlerRelayMsgSettingsWrite(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SETTINGS_WRITE, state, "settings_write") {}
};

class SBPCallbackHandlerRelayMsgSettingsWriteResp
    : public SBPCallbackHandlerRelay<msg_settings_write_resp_t,
                                     piksi_multi_msgs::MsgSettingsWriteResp> {
  public:
    inline SBPCallbackHandlerRelayMsgSettingsWriteResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SETTINGS_WRITE_RESP, state, "settings_write_resp") {}
};

class SBPCallbackHandlerRelayMsgSettingsReadReq
    : public SBPCallbackHandlerRelay<msg_settings_read_req_t,
                                     piksi_multi_msgs::MsgSettingsReadReq> {
  public:
    inline SBPCallbackHandlerRelayMsgSettingsReadReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SETTINGS_READ_REQ, state, "settings_read_req") {}
};

class SBPCallbackHandlerRelayMsgSettingsReadResp
    : public SBPCallbackHandlerRelay<msg_settings_read_resp_t,
                                     piksi_multi_msgs::MsgSettingsReadResp> {
  public:
    inline SBPCallbackHandlerRelayMsgSettingsReadResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SETTINGS_READ_RESP, state, "settings_read_resp") {}
};

class SBPCallbackHandlerRelayMsgSettingsReadByIndexReq
    : public SBPCallbackHandlerRelay<msg_settings_read_by_index_req_t,
                                     piksi_multi_msgs::MsgSettingsReadByIndexReq> {
  public:
    inline SBPCallbackHandlerRelayMsgSettingsReadByIndexReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SETTINGS_READ_BY_INDEX_REQ, state, "settings_read_by_index_req") {}
};

class SBPCallbackHandlerRelayMsgSettingsReadByIndexResp
    : public SBPCallbackHandlerRelay<msg_settings_read_by_index_resp_t,
                                     piksi_multi_msgs::MsgSettingsReadByIndexResp> {
  public:
    inline SBPCallbackHandlerRelayMsgSettingsReadByIndexResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SETTINGS_READ_BY_INDEX_RESP, state, "settings_read_by_index_resp") {}
};

class SBPCallbackHandlerRelayMsgSettingsRegister
    : public SBPCallbackHandlerRelay<msg_settings_register_t,
                                     piksi_multi_msgs::MsgSettingsRegister> {
  public:
    inline SBPCallbackHandlerRelayMsgSettingsRegister(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SETTINGS_REGISTER, state, "settings_register") {}
};

class SBPCallbackHandlerRelayMsgSettingsRegisterResp
    : public SBPCallbackHandlerRelay<msg_settings_register_resp_t,
                                     piksi_multi_msgs::MsgSettingsRegisterResp> {
  public:
    inline SBPCallbackHandlerRelayMsgSettingsRegisterResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SETTINGS_REGISTER_RESP, state, "settings_register_resp") {}
};

class SBPCallbackHandlerRelayMsgSsrOrbitClock
    : public SBPCallbackHandlerRelay<msg_ssr_orbit_clock_t,
                                     piksi_multi_msgs::MsgSsrOrbitClock> {
  public:
    inline SBPCallbackHandlerRelayMsgSsrOrbitClock(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SSR_ORBIT_CLOCK, state, "ssr_orbit_clock") {}
};

class SBPCallbackHandlerRelayMsgSsrCodeBiases
    : public SBPCallbackHandlerRelay<msg_ssr_code_biases_t,
                                     piksi_multi_msgs::MsgSsrCodeBiases> {
  public:
    inline SBPCallbackHandlerRelayMsgSsrCodeBiases(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SSR_CODE_BIASES, state, "ssr_code_biases") {}
};

class SBPCallbackHandlerRelayMsgSsrPhaseBiases
    : public SBPCallbackHandlerRelay<msg_ssr_phase_biases_t,
                                     piksi_multi_msgs::MsgSsrPhaseBiases> {
  public:
    inline SBPCallbackHandlerRelayMsgSsrPhaseBiases(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SSR_PHASE_BIASES, state, "ssr_phase_biases") {}
};

class SBPCallbackHandlerRelayMsgSsrStecCorrection
    : public SBPCallbackHandlerRelay<msg_ssr_stec_correction_t,
                                     piksi_multi_msgs::MsgSsrStecCorrection> {
  public:
    inline SBPCallbackHandlerRelayMsgSsrStecCorrection(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SSR_STEC_CORRECTION, state, "ssr_stec_correction") {}
};

class SBPCallbackHandlerRelayMsgSsrGriddedCorrection
    : public SBPCallbackHandlerRelay<msg_ssr_gridded_correction_t,
                                     piksi_multi_msgs::MsgSsrGriddedCorrection> {
  public:
    inline SBPCallbackHandlerRelayMsgSsrGriddedCorrection(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SSR_GRIDDED_CORRECTION, state, "ssr_gridded_correction") {}
};

class SBPCallbackHandlerRelayMsgSsrGridDefinition
    : public SBPCallbackHandlerRelay<msg_ssr_grid_definition_t,
                                     piksi_multi_msgs::MsgSsrGridDefinition> {
  public:
    inline SBPCallbackHandlerRelayMsgSsrGridDefinition(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_SSR_GRID_DEFINITION, state, "ssr_grid_definition") {}
};

class SBPCallbackHandlerRelayMsgStartup
    : public SBPCallbackHandlerRelay<msg_startup_t,
                                     piksi_multi_msgs::MsgStartup> {
  public:
    inline SBPCallbackHandlerRelayMsgStartup(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_STARTUP, state, "startup") {}
};

class SBPCallbackHandlerRelayMsgDgnssStatus
    : public SBPCallbackHandlerRelay<msg_dgnss_status_t,
                                     piksi_multi_msgs::MsgDgnssStatus> {
  public:
    inline SBPCallbackHandlerRelayMsgDgnssStatus(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_DGNSS_STATUS, state, "dgnss_status") {}
};

class SBPCallbackHandlerRelayMsgHeartbeat
    : public SBPCallbackHandlerRelay<msg_heartbeat_t,
                                     piksi_multi_msgs::MsgHeartbeat> {
  public:
    inline SBPCallbackHandlerRelayMsgHeartbeat(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_HEARTBEAT, state, "heartbeat") {}
};

class SBPCallbackHandlerRelayMsgInsStatus
    : public SBPCallbackHandlerRelay<msg_ins_status_t,
                                     piksi_multi_msgs::MsgInsStatus> {
  public:
    inline SBPCallbackHandlerRelayMsgInsStatus(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_INS_STATUS, state, "ins_status") {}
};

class SBPCallbackHandlerRelayMsgCsacTelemetry
    : public SBPCallbackHandlerRelay<msg_csac_telemetry_t,
                                     piksi_multi_msgs::MsgCsacTelemetry> {
  public:
    inline SBPCallbackHandlerRelayMsgCsacTelemetry(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_CSAC_TELEMETRY, state, "csac_telemetry") {}
};

class SBPCallbackHandlerRelayMsgCsacTelemetryLabels
    : public SBPCallbackHandlerRelay<msg_csac_telemetry_labels_t,
                                     piksi_multi_msgs::MsgCsacTelemetryLabels> {
  public:
    inline SBPCallbackHandlerRelayMsgCsacTelemetryLabels(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_CSAC_TELEMETRY_LABELS, state, "csac_telemetry_labels") {}
};

class SBPCallbackHandlerRelayMsgTrackingState
    : public SBPCallbackHandlerRelay<msg_tracking_state_t,
                                     piksi_multi_msgs::MsgTrackingState> {
  public:
    inline SBPCallbackHandlerRelayMsgTrackingState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_TRACKING_STATE, state, "tracking_state") {}
};

class SBPCallbackHandlerRelayMsgMeasurementState
    : public SBPCallbackHandlerRelay<msg_measurement_state_t,
                                     piksi_multi_msgs::MsgMeasurementState> {
  public:
    inline SBPCallbackHandlerRelayMsgMeasurementState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_MEASUREMENT_STATE, state, "measurement_state") {}
};

class SBPCallbackHandlerRelayMsgTrackingIq
    : public SBPCallbackHandlerRelay<msg_tracking_iq_t,
                                     piksi_multi_msgs::MsgTrackingIq> {
  public:
    inline SBPCallbackHandlerRelayMsgTrackingIq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_TRACKING_IQ, state, "tracking_iq") {}
};

class SBPCallbackHandlerRelayMsgUserData
    : public SBPCallbackHandlerRelay<msg_user_data_t,
                                     piksi_multi_msgs::MsgUserData> {
  public:
    inline SBPCallbackHandlerRelayMsgUserData(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_USER_DATA, state, "user_data") {}
};

class SBPCallbackHandlerRelayMsgOdometry
    : public SBPCallbackHandlerRelay<msg_odometry_t,
                                     piksi_multi_msgs::MsgOdometry> {
  public:
    inline SBPCallbackHandlerRelayMsgOdometry(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SBPCallbackHandlerRelay(nh, SBP_MSG_ODOMETRY, state, "odometry") {}
};

// Create all relays
std::vector<SBPCallbackHandler::Ptr> createAllSbpMsgRelays(const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  std::vector<SBPCallbackHandler::Ptr> relays;

  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgAcqResult(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgAcqSvProfile(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgBootloaderHandshakeResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgBootloaderJumpToApp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgNapDeviceDnaResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgExtEvent(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFileioReadReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFileioReadResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFileioReadDirReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFileioReadDirResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFileioRemove(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFileioWriteReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFileioWriteResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFileioConfigReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFileioConfigResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFlashProgram(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFlashDone(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFlashReadReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFlashReadResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFlashErase(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgStmFlashLockSector(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgStmFlashUnlockSector(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgStmUniqueIdResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgM25FlashWriteStatus(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgImuRaw(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgImuAux(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgLinuxCpuState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgLinuxMemState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgLinuxSysState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgLinuxProcessSocketCounts(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgLinuxProcessSocketQueues(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgLinuxSocketUsage(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgLinuxProcessFdCount(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgLinuxProcessFdSummary(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgLog(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFwd(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgMagRaw(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgGpsTime(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgUtcTime(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgDops(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgPosEcef(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgPosEcefCov(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgPosLlh(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgPosLlhCov(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgBaselineEcef(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgBaselineNed(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgVelEcef(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgVelEcefCov(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgVelNed(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgVelNedCov(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgVelBody(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgAgeCorrections(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgNdbEvent(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgObs(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgBasePosLlh(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgBasePosEcef(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgEphemerisGps(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgEphemerisQzss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgEphemerisBds(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgEphemerisGal(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgEphemerisSbas(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgEphemerisGlo(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgIono(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgGnssCapb(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgGroupDelay(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgAlmanacGps(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgAlmanacGlo(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgGloBiases(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSvAzEl(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgOsr(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgBaselineHeading(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgOrientQuat(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgOrientEuler(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgAngularRate(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgReset(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgResetFilters(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgThreadState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgUartState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgIarState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgMaskSatellite(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgDeviceMonitor(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgCommandReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgCommandResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgCommandOutput(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgNetworkStateResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgNetworkBandwidthUsage(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgCellModemStatus(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSpecan(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgFrontEndGain(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSbasRaw(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSettingsWrite(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSettingsWriteResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSettingsReadReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSettingsReadResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSettingsReadByIndexReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSettingsReadByIndexResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSettingsRegister(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSettingsRegisterResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSsrOrbitClock(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSsrCodeBiases(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSsrPhaseBiases(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSsrStecCorrection(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSsrGriddedCorrection(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgSsrGridDefinition(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgStartup(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgDgnssStatus(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgHeartbeat(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgInsStatus(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgCsacTelemetry(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgCsacTelemetryLabels(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgTrackingState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgMeasurementState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgTrackingIq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgUserData(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayMsgOdometry(nh, state)));
  

  // Remove all invalid (nullptr) callbacks.
  relays.erase(std::remove_if(
  relays.begin(), relays.end(),
  [](const SBPCallbackHandler::Ptr& relay) { return relay.get() == nullptr; }));

  return relays;
}

} // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_SBP_CALLBACK_HANDLER_RELAY_SBP_H_