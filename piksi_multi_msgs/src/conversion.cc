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

#include "piksi_multi_msgs/conversion.h"

namespace piksi_multi_msgs {

MsgAcqResult convertSbpMsgToRosMsg(const msg_acq_result_t& sbp_msg)
{
  MsgAcqResult ros_msg;
  ros_msg.cn0 = sbp_msg.cn0;
  ros_msg.cp = sbp_msg.cp;
  ros_msg.cf = sbp_msg.cf;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  return ros_msg;
}

AcqSvProfile convertSbpMsgToRosMsg(const acq_sv_profile_t& sbp_msg)
{
  AcqSvProfile ros_msg;
  ros_msg.job_type = sbp_msg.job_type;
  ros_msg.status = sbp_msg.status;
  ros_msg.cn0 = sbp_msg.cn0;
  ros_msg.int_time = sbp_msg.int_time;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.bin_width = sbp_msg.bin_width;
  ros_msg.timestamp = sbp_msg.timestamp;
  ros_msg.time_spent = sbp_msg.time_spent;
  ros_msg.cf_min = sbp_msg.cf_min;
  ros_msg.cf_max = sbp_msg.cf_max;
  ros_msg.cf = sbp_msg.cf;
  ros_msg.cp = sbp_msg.cp;
  return ros_msg;
}

MsgAcqSvProfile convertSbpMsgToRosMsg(const msg_acq_sv_profile_t& sbp_msg)
{
  MsgAcqSvProfile ros_msg;
  for (auto msg : sbp_msg.acq_sv_profile)
    ros_msg.acq_sv_profile.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MsgBootloaderHandshakeResp convertSbpMsgToRosMsg(const msg_bootloader_handshake_resp_t& sbp_msg)
{
  MsgBootloaderHandshakeResp ros_msg;
  ros_msg.flags = sbp_msg.flags;
  ros_msg.version = sbp_msg.version;
  return ros_msg;
}

MsgBootloaderJumpToApp convertSbpMsgToRosMsg(const msg_bootloader_jump_to_app_t& sbp_msg)
{
  MsgBootloaderJumpToApp ros_msg;
  ros_msg.jump = sbp_msg.jump;
  return ros_msg;
}

MsgNapDeviceDnaResp convertSbpMsgToRosMsg(const msg_nap_device_dna_resp_t& sbp_msg)
{
  MsgNapDeviceDnaResp ros_msg;
  for (auto msg : sbp_msg.dna)
    ros_msg.dna.push_back(msg);
  return ros_msg;
}

MsgExtEvent convertSbpMsgToRosMsg(const msg_ext_event_t& sbp_msg)
{
  MsgExtEvent ros_msg;
  ros_msg.wn = sbp_msg.wn;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.ns_residual = sbp_msg.ns_residual;
  ros_msg.flags = sbp_msg.flags;
  ros_msg.pin = sbp_msg.pin;
  return ros_msg;
}

MsgFileioReadReq convertSbpMsgToRosMsg(const msg_fileio_read_req_t& sbp_msg)
{
  MsgFileioReadReq ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  ros_msg.offset = sbp_msg.offset;
  ros_msg.chunk_size = sbp_msg.chunk_size;
  ros_msg.filename = sbp_msg.filename;
  return ros_msg;
}

MsgFileioReadResp convertSbpMsgToRosMsg(const msg_fileio_read_resp_t& sbp_msg)
{
  MsgFileioReadResp ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  for (auto msg : sbp_msg.contents)
    ros_msg.contents.push_back(msg);
  return ros_msg;
}

MsgFileioReadDirReq convertSbpMsgToRosMsg(const msg_fileio_read_dir_req_t& sbp_msg)
{
  MsgFileioReadDirReq ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  ros_msg.offset = sbp_msg.offset;
  ros_msg.dirname = sbp_msg.dirname;
  return ros_msg;
}

MsgFileioReadDirResp convertSbpMsgToRosMsg(const msg_fileio_read_dir_resp_t& sbp_msg)
{
  MsgFileioReadDirResp ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  for (auto msg : sbp_msg.contents)
    ros_msg.contents.push_back(msg);
  return ros_msg;
}

MsgFileioRemove convertSbpMsgToRosMsg(const msg_fileio_remove_t& sbp_msg)
{
  MsgFileioRemove ros_msg;
  ros_msg.filename = sbp_msg.filename;
  return ros_msg;
}

MsgFileioWriteReq convertSbpMsgToRosMsg(const msg_fileio_write_req_t& sbp_msg)
{
  MsgFileioWriteReq ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  ros_msg.offset = sbp_msg.offset;
  ros_msg.filename = sbp_msg.filename;
  for (auto msg : sbp_msg.data)
    ros_msg.data.push_back(msg);
  return ros_msg;
}

MsgFileioWriteResp convertSbpMsgToRosMsg(const msg_fileio_write_resp_t& sbp_msg)
{
  MsgFileioWriteResp ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  return ros_msg;
}

MsgFileioConfigReq convertSbpMsgToRosMsg(const msg_fileio_config_req_t& sbp_msg)
{
  MsgFileioConfigReq ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  return ros_msg;
}

MsgFileioConfigResp convertSbpMsgToRosMsg(const msg_fileio_config_resp_t& sbp_msg)
{
  MsgFileioConfigResp ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  ros_msg.window_size = sbp_msg.window_size;
  ros_msg.batch_size = sbp_msg.batch_size;
  ros_msg.fileio_version = sbp_msg.fileio_version;
  return ros_msg;
}

MsgFlashProgram convertSbpMsgToRosMsg(const msg_flash_program_t& sbp_msg)
{
  MsgFlashProgram ros_msg;
  ros_msg.target = sbp_msg.target;
  for (auto msg : sbp_msg.addr_start)
    ros_msg.addr_start.push_back(msg);
  ros_msg.addr_len = sbp_msg.addr_len;
  for (auto msg : sbp_msg.data)
    ros_msg.data.push_back(msg);
  return ros_msg;
}

MsgFlashDone convertSbpMsgToRosMsg(const msg_flash_done_t& sbp_msg)
{
  MsgFlashDone ros_msg;
  ros_msg.response = sbp_msg.response;
  return ros_msg;
}

MsgFlashReadReq convertSbpMsgToRosMsg(const msg_flash_read_req_t& sbp_msg)
{
  MsgFlashReadReq ros_msg;
  ros_msg.target = sbp_msg.target;
  for (auto msg : sbp_msg.addr_start)
    ros_msg.addr_start.push_back(msg);
  ros_msg.addr_len = sbp_msg.addr_len;
  return ros_msg;
}

MsgFlashReadResp convertSbpMsgToRosMsg(const msg_flash_read_resp_t& sbp_msg)
{
  MsgFlashReadResp ros_msg;
  ros_msg.target = sbp_msg.target;
  for (auto msg : sbp_msg.addr_start)
    ros_msg.addr_start.push_back(msg);
  ros_msg.addr_len = sbp_msg.addr_len;
  return ros_msg;
}

MsgFlashErase convertSbpMsgToRosMsg(const msg_flash_erase_t& sbp_msg)
{
  MsgFlashErase ros_msg;
  ros_msg.target = sbp_msg.target;
  ros_msg.sector_num = sbp_msg.sector_num;
  return ros_msg;
}

MsgStmFlashLockSector convertSbpMsgToRosMsg(const msg_stm_flash_lock_sector_t& sbp_msg)
{
  MsgStmFlashLockSector ros_msg;
  ros_msg.sector = sbp_msg.sector;
  return ros_msg;
}

MsgStmFlashUnlockSector convertSbpMsgToRosMsg(const msg_stm_flash_unlock_sector_t& sbp_msg)
{
  MsgStmFlashUnlockSector ros_msg;
  ros_msg.sector = sbp_msg.sector;
  return ros_msg;
}

MsgStmUniqueIdResp convertSbpMsgToRosMsg(const msg_stm_unique_id_resp_t& sbp_msg)
{
  MsgStmUniqueIdResp ros_msg;
  for (auto msg : sbp_msg.stm_id)
    ros_msg.stm_id.push_back(msg);
  return ros_msg;
}

MsgM25FlashWriteStatus convertSbpMsgToRosMsg(const msg_m25_flash_write_status_t& sbp_msg)
{
  MsgM25FlashWriteStatus ros_msg;
  for (auto msg : sbp_msg.status)
    ros_msg.status.push_back(msg);
  return ros_msg;
}

GnssSignal convertSbpMsgToRosMsg(const sbp_gnss_signal_t& sbp_msg)
{
  GnssSignal ros_msg;
  ros_msg.sat = sbp_msg.sat;
  ros_msg.code = sbp_msg.code;
  return ros_msg;
}

SvId convertSbpMsgToRosMsg(const sv_id_t& sbp_msg)
{
  SvId ros_msg;
  ros_msg.satId = sbp_msg.satId;
  ros_msg.constellation = sbp_msg.constellation;
  return ros_msg;
}

GpsTimeSec convertSbpMsgToRosMsg(const gps_time_sec_t& sbp_msg)
{
  GpsTimeSec ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.wn = sbp_msg.wn;
  return ros_msg;
}

GpsTime convertSbpMsgToRosMsg(const sbp_gps_time_t& sbp_msg)
{
  GpsTime ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.ns_residual = sbp_msg.ns_residual;
  ros_msg.wn = sbp_msg.wn;
  return ros_msg;
}

CarrierPhase convertSbpMsgToRosMsg(const carrier_phase_t& sbp_msg)
{
  CarrierPhase ros_msg;
  ros_msg.i = sbp_msg.i;
  ros_msg.f = sbp_msg.f;
  return ros_msg;
}

MsgImuRaw convertSbpMsgToRosMsg(const msg_imu_raw_t& sbp_msg)
{
  MsgImuRaw ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.tow_f = sbp_msg.tow_f;
  ros_msg.acc_x = sbp_msg.acc_x;
  ros_msg.acc_y = sbp_msg.acc_y;
  ros_msg.acc_z = sbp_msg.acc_z;
  ros_msg.gyr_x = sbp_msg.gyr_x;
  ros_msg.gyr_y = sbp_msg.gyr_y;
  ros_msg.gyr_z = sbp_msg.gyr_z;
  return ros_msg;
}

MsgImuAux convertSbpMsgToRosMsg(const msg_imu_aux_t& sbp_msg)
{
  MsgImuAux ros_msg;
  ros_msg.imu_type = sbp_msg.imu_type;
  ros_msg.temp = sbp_msg.temp;
  ros_msg.imu_conf = sbp_msg.imu_conf;
  return ros_msg;
}

MsgLinuxCpuState convertSbpMsgToRosMsg(const msg_linux_cpu_state_t& sbp_msg)
{
  MsgLinuxCpuState ros_msg;
  ros_msg.index = sbp_msg.index;
  ros_msg.pid = sbp_msg.pid;
  ros_msg.pcpu = sbp_msg.pcpu;
  ros_msg.tname = sbp_msg.tname;
  ros_msg.cmdline = sbp_msg.cmdline;
  return ros_msg;
}

MsgLinuxMemState convertSbpMsgToRosMsg(const msg_linux_mem_state_t& sbp_msg)
{
  MsgLinuxMemState ros_msg;
  ros_msg.index = sbp_msg.index;
  ros_msg.pid = sbp_msg.pid;
  ros_msg.pmem = sbp_msg.pmem;
  ros_msg.tname = sbp_msg.tname;
  ros_msg.cmdline = sbp_msg.cmdline;
  return ros_msg;
}

MsgLinuxSysState convertSbpMsgToRosMsg(const msg_linux_sys_state_t& sbp_msg)
{
  MsgLinuxSysState ros_msg;
  ros_msg.mem_total = sbp_msg.mem_total;
  ros_msg.pcpu = sbp_msg.pcpu;
  ros_msg.pmem = sbp_msg.pmem;
  ros_msg.procs_starting = sbp_msg.procs_starting;
  ros_msg.procs_stopping = sbp_msg.procs_stopping;
  ros_msg.pid_count = sbp_msg.pid_count;
  return ros_msg;
}

MsgLinuxProcessSocketCounts convertSbpMsgToRosMsg(const msg_linux_process_socket_counts_t& sbp_msg)
{
  MsgLinuxProcessSocketCounts ros_msg;
  ros_msg.index = sbp_msg.index;
  ros_msg.pid = sbp_msg.pid;
  ros_msg.socket_count = sbp_msg.socket_count;
  ros_msg.socket_types = sbp_msg.socket_types;
  ros_msg.socket_states = sbp_msg.socket_states;
  ros_msg.cmdline = sbp_msg.cmdline;
  return ros_msg;
}

MsgLinuxProcessSocketQueues convertSbpMsgToRosMsg(const msg_linux_process_socket_queues_t& sbp_msg)
{
  MsgLinuxProcessSocketQueues ros_msg;
  ros_msg.index = sbp_msg.index;
  ros_msg.pid = sbp_msg.pid;
  ros_msg.recv_queued = sbp_msg.recv_queued;
  ros_msg.send_queued = sbp_msg.send_queued;
  ros_msg.socket_types = sbp_msg.socket_types;
  ros_msg.socket_states = sbp_msg.socket_states;
  ros_msg.address_of_largest = sbp_msg.address_of_largest;
  ros_msg.cmdline = sbp_msg.cmdline;
  return ros_msg;
}

MsgLinuxSocketUsage convertSbpMsgToRosMsg(const msg_linux_socket_usage_t& sbp_msg)
{
  MsgLinuxSocketUsage ros_msg;
  ros_msg.avg_queue_depth = sbp_msg.avg_queue_depth;
  ros_msg.max_queue_depth = sbp_msg.max_queue_depth;
  for (auto msg : sbp_msg.socket_state_counts)
    ros_msg.socket_state_counts.push_back(msg);
  for (auto msg : sbp_msg.socket_type_counts)
    ros_msg.socket_type_counts.push_back(msg);
  return ros_msg;
}

MsgLinuxProcessFdCount convertSbpMsgToRosMsg(const msg_linux_process_fd_count_t& sbp_msg)
{
  MsgLinuxProcessFdCount ros_msg;
  ros_msg.index = sbp_msg.index;
  ros_msg.pid = sbp_msg.pid;
  ros_msg.fd_count = sbp_msg.fd_count;
  ros_msg.cmdline = sbp_msg.cmdline;
  return ros_msg;
}

MsgLinuxProcessFdSummary convertSbpMsgToRosMsg(const msg_linux_process_fd_summary_t& sbp_msg)
{
  MsgLinuxProcessFdSummary ros_msg;
  ros_msg.sys_fd_count = sbp_msg.sys_fd_count;
  ros_msg.most_opened = sbp_msg.most_opened;
  return ros_msg;
}

MsgLog convertSbpMsgToRosMsg(const msg_log_t& sbp_msg)
{
  MsgLog ros_msg;
  ros_msg.level = sbp_msg.level;
  ros_msg.text = sbp_msg.text;
  return ros_msg;
}

MsgFwd convertSbpMsgToRosMsg(const msg_fwd_t& sbp_msg)
{
  MsgFwd ros_msg;
  ros_msg.source = sbp_msg.source;
  ros_msg.protocol = sbp_msg.protocol;
  ros_msg.fwd_payload = sbp_msg.fwd_payload;
  return ros_msg;
}

MsgMagRaw convertSbpMsgToRosMsg(const msg_mag_raw_t& sbp_msg)
{
  MsgMagRaw ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.tow_f = sbp_msg.tow_f;
  ros_msg.mag_x = sbp_msg.mag_x;
  ros_msg.mag_y = sbp_msg.mag_y;
  ros_msg.mag_z = sbp_msg.mag_z;
  return ros_msg;
}

MsgGpsTime convertSbpMsgToRosMsg(const msg_gps_time_t& sbp_msg)
{
  MsgGpsTime ros_msg;
  ros_msg.wn = sbp_msg.wn;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.ns_residual = sbp_msg.ns_residual;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgUtcTime convertSbpMsgToRosMsg(const msg_utc_time_t& sbp_msg)
{
  MsgUtcTime ros_msg;
  ros_msg.flags = sbp_msg.flags;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.year = sbp_msg.year;
  ros_msg.month = sbp_msg.month;
  ros_msg.day = sbp_msg.day;
  ros_msg.hours = sbp_msg.hours;
  ros_msg.minutes = sbp_msg.minutes;
  ros_msg.seconds = sbp_msg.seconds;
  ros_msg.ns = sbp_msg.ns;
  return ros_msg;
}

MsgDops convertSbpMsgToRosMsg(const msg_dops_t& sbp_msg)
{
  MsgDops ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.gdop = sbp_msg.gdop;
  ros_msg.pdop = sbp_msg.pdop;
  ros_msg.tdop = sbp_msg.tdop;
  ros_msg.hdop = sbp_msg.hdop;
  ros_msg.vdop = sbp_msg.vdop;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgPosEcef convertSbpMsgToRosMsg(const msg_pos_ecef_t& sbp_msg)
{
  MsgPosEcef ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.x = sbp_msg.x;
  ros_msg.y = sbp_msg.y;
  ros_msg.z = sbp_msg.z;
  ros_msg.accuracy = sbp_msg.accuracy;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgPosEcefCov convertSbpMsgToRosMsg(const msg_pos_ecef_cov_t& sbp_msg)
{
  MsgPosEcefCov ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.x = sbp_msg.x;
  ros_msg.y = sbp_msg.y;
  ros_msg.z = sbp_msg.z;
  ros_msg.cov_x_x = sbp_msg.cov_x_x;
  ros_msg.cov_x_y = sbp_msg.cov_x_y;
  ros_msg.cov_x_z = sbp_msg.cov_x_z;
  ros_msg.cov_y_y = sbp_msg.cov_y_y;
  ros_msg.cov_y_z = sbp_msg.cov_y_z;
  ros_msg.cov_z_z = sbp_msg.cov_z_z;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgPosLlh convertSbpMsgToRosMsg(const msg_pos_llh_t& sbp_msg)
{
  MsgPosLlh ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.lat = sbp_msg.lat;
  ros_msg.lon = sbp_msg.lon;
  ros_msg.height = sbp_msg.height;
  ros_msg.h_accuracy = sbp_msg.h_accuracy;
  ros_msg.v_accuracy = sbp_msg.v_accuracy;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgPosLlhCov convertSbpMsgToRosMsg(const msg_pos_llh_cov_t& sbp_msg)
{
  MsgPosLlhCov ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.lat = sbp_msg.lat;
  ros_msg.lon = sbp_msg.lon;
  ros_msg.height = sbp_msg.height;
  ros_msg.cov_n_n = sbp_msg.cov_n_n;
  ros_msg.cov_n_e = sbp_msg.cov_n_e;
  ros_msg.cov_n_d = sbp_msg.cov_n_d;
  ros_msg.cov_e_e = sbp_msg.cov_e_e;
  ros_msg.cov_e_d = sbp_msg.cov_e_d;
  ros_msg.cov_d_d = sbp_msg.cov_d_d;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgBaselineEcef convertSbpMsgToRosMsg(const msg_baseline_ecef_t& sbp_msg)
{
  MsgBaselineEcef ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.x = sbp_msg.x;
  ros_msg.y = sbp_msg.y;
  ros_msg.z = sbp_msg.z;
  ros_msg.accuracy = sbp_msg.accuracy;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgBaselineNed convertSbpMsgToRosMsg(const msg_baseline_ned_t& sbp_msg)
{
  MsgBaselineNed ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.n = sbp_msg.n;
  ros_msg.e = sbp_msg.e;
  ros_msg.d = sbp_msg.d;
  ros_msg.h_accuracy = sbp_msg.h_accuracy;
  ros_msg.v_accuracy = sbp_msg.v_accuracy;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgVelEcef convertSbpMsgToRosMsg(const msg_vel_ecef_t& sbp_msg)
{
  MsgVelEcef ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.x = sbp_msg.x;
  ros_msg.y = sbp_msg.y;
  ros_msg.z = sbp_msg.z;
  ros_msg.accuracy = sbp_msg.accuracy;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgVelEcefCov convertSbpMsgToRosMsg(const msg_vel_ecef_cov_t& sbp_msg)
{
  MsgVelEcefCov ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.x = sbp_msg.x;
  ros_msg.y = sbp_msg.y;
  ros_msg.z = sbp_msg.z;
  ros_msg.cov_x_x = sbp_msg.cov_x_x;
  ros_msg.cov_x_y = sbp_msg.cov_x_y;
  ros_msg.cov_x_z = sbp_msg.cov_x_z;
  ros_msg.cov_y_y = sbp_msg.cov_y_y;
  ros_msg.cov_y_z = sbp_msg.cov_y_z;
  ros_msg.cov_z_z = sbp_msg.cov_z_z;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgVelNed convertSbpMsgToRosMsg(const msg_vel_ned_t& sbp_msg)
{
  MsgVelNed ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.n = sbp_msg.n;
  ros_msg.e = sbp_msg.e;
  ros_msg.d = sbp_msg.d;
  ros_msg.h_accuracy = sbp_msg.h_accuracy;
  ros_msg.v_accuracy = sbp_msg.v_accuracy;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgVelNedCov convertSbpMsgToRosMsg(const msg_vel_ned_cov_t& sbp_msg)
{
  MsgVelNedCov ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.n = sbp_msg.n;
  ros_msg.e = sbp_msg.e;
  ros_msg.d = sbp_msg.d;
  ros_msg.cov_n_n = sbp_msg.cov_n_n;
  ros_msg.cov_n_e = sbp_msg.cov_n_e;
  ros_msg.cov_n_d = sbp_msg.cov_n_d;
  ros_msg.cov_e_e = sbp_msg.cov_e_e;
  ros_msg.cov_e_d = sbp_msg.cov_e_d;
  ros_msg.cov_d_d = sbp_msg.cov_d_d;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgVelBody convertSbpMsgToRosMsg(const msg_vel_body_t& sbp_msg)
{
  MsgVelBody ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.x = sbp_msg.x;
  ros_msg.y = sbp_msg.y;
  ros_msg.z = sbp_msg.z;
  ros_msg.cov_x_x = sbp_msg.cov_x_x;
  ros_msg.cov_x_y = sbp_msg.cov_x_y;
  ros_msg.cov_x_z = sbp_msg.cov_x_z;
  ros_msg.cov_y_y = sbp_msg.cov_y_y;
  ros_msg.cov_y_z = sbp_msg.cov_y_z;
  ros_msg.cov_z_z = sbp_msg.cov_z_z;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgAgeCorrections convertSbpMsgToRosMsg(const msg_age_corrections_t& sbp_msg)
{
  MsgAgeCorrections ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.age = sbp_msg.age;
  return ros_msg;
}

MsgNdbEvent convertSbpMsgToRosMsg(const msg_ndb_event_t& sbp_msg)
{
  MsgNdbEvent ros_msg;
  ros_msg.recv_time = sbp_msg.recv_time;
  ros_msg.event = sbp_msg.event;
  ros_msg.object_type = sbp_msg.object_type;
  ros_msg.result = sbp_msg.result;
  ros_msg.data_source = sbp_msg.data_source;
  ros_msg.object_sid = convertSbpMsgToRosMsg(sbp_msg.object_sid);
  ros_msg.src_sid = convertSbpMsgToRosMsg(sbp_msg.src_sid);
  ros_msg.original_sender = sbp_msg.original_sender;
  return ros_msg;
}

ObservationHeader convertSbpMsgToRosMsg(const observation_header_t& sbp_msg)
{
  ObservationHeader ros_msg;
  ros_msg.t = convertSbpMsgToRosMsg(sbp_msg.t);
  ros_msg.n_obs = sbp_msg.n_obs;
  return ros_msg;
}

Doppler convertSbpMsgToRosMsg(const doppler_t& sbp_msg)
{
  Doppler ros_msg;
  ros_msg.i = sbp_msg.i;
  ros_msg.f = sbp_msg.f;
  return ros_msg;
}

PackedObsContent convertSbpMsgToRosMsg(const packed_obs_content_t& sbp_msg)
{
  PackedObsContent ros_msg;
  ros_msg.P = sbp_msg.P;
  ros_msg.L = convertSbpMsgToRosMsg(sbp_msg.L);
  ros_msg.D = convertSbpMsgToRosMsg(sbp_msg.D);
  ros_msg.cn0 = sbp_msg.cn0;
  ros_msg.lock = sbp_msg.lock;
  ros_msg.flags = sbp_msg.flags;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  return ros_msg;
}

PackedOsrContent convertSbpMsgToRosMsg(const packed_osr_content_t& sbp_msg)
{
  PackedOsrContent ros_msg;
  ros_msg.P = sbp_msg.P;
  ros_msg.L = convertSbpMsgToRosMsg(sbp_msg.L);
  ros_msg.lock = sbp_msg.lock;
  ros_msg.flags = sbp_msg.flags;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.iono_std = sbp_msg.iono_std;
  ros_msg.tropo_std = sbp_msg.tropo_std;
  ros_msg.range_std = sbp_msg.range_std;
  return ros_msg;
}

MsgObs convertSbpMsgToRosMsg(const msg_obs_t& sbp_msg)
{
  MsgObs ros_msg;
  ros_msg.header = convertSbpMsgToRosMsg(sbp_msg.header);
  for (auto msg : sbp_msg.obs)
    ros_msg.obs.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MsgBasePosLlh convertSbpMsgToRosMsg(const msg_base_pos_llh_t& sbp_msg)
{
  MsgBasePosLlh ros_msg;
  ros_msg.lat = sbp_msg.lat;
  ros_msg.lon = sbp_msg.lon;
  ros_msg.height = sbp_msg.height;
  return ros_msg;
}

MsgBasePosEcef convertSbpMsgToRosMsg(const msg_base_pos_ecef_t& sbp_msg)
{
  MsgBasePosEcef ros_msg;
  ros_msg.x = sbp_msg.x;
  ros_msg.y = sbp_msg.y;
  ros_msg.z = sbp_msg.z;
  return ros_msg;
}

EphemerisCommonContent convertSbpMsgToRosMsg(const ephemeris_common_content_t& sbp_msg)
{
  EphemerisCommonContent ros_msg;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.toe = convertSbpMsgToRosMsg(sbp_msg.toe);
  ros_msg.ura = sbp_msg.ura;
  ros_msg.fit_interval = sbp_msg.fit_interval;
  ros_msg.valid = sbp_msg.valid;
  ros_msg.health_bits = sbp_msg.health_bits;
  return ros_msg;
}

MsgEphemerisGps convertSbpMsgToRosMsg(const msg_ephemeris_gps_t& sbp_msg)
{
  MsgEphemerisGps ros_msg;
  ros_msg.common = convertSbpMsgToRosMsg(sbp_msg.common);
  ros_msg.tgd = sbp_msg.tgd;
  ros_msg.c_rs = sbp_msg.c_rs;
  ros_msg.c_rc = sbp_msg.c_rc;
  ros_msg.c_uc = sbp_msg.c_uc;
  ros_msg.c_us = sbp_msg.c_us;
  ros_msg.c_ic = sbp_msg.c_ic;
  ros_msg.c_is = sbp_msg.c_is;
  ros_msg.dn = sbp_msg.dn;
  ros_msg.m0 = sbp_msg.m0;
  ros_msg.ecc = sbp_msg.ecc;
  ros_msg.sqrta = sbp_msg.sqrta;
  ros_msg.omega0 = sbp_msg.omega0;
  ros_msg.omegadot = sbp_msg.omegadot;
  ros_msg.w = sbp_msg.w;
  ros_msg.inc = sbp_msg.inc;
  ros_msg.inc_dot = sbp_msg.inc_dot;
  ros_msg.af0 = sbp_msg.af0;
  ros_msg.af1 = sbp_msg.af1;
  ros_msg.af2 = sbp_msg.af2;
  ros_msg.toc = convertSbpMsgToRosMsg(sbp_msg.toc);
  ros_msg.iode = sbp_msg.iode;
  ros_msg.iodc = sbp_msg.iodc;
  return ros_msg;
}

MsgEphemerisQzss convertSbpMsgToRosMsg(const msg_ephemeris_qzss_t& sbp_msg)
{
  MsgEphemerisQzss ros_msg;
  ros_msg.common = convertSbpMsgToRosMsg(sbp_msg.common);
  ros_msg.tgd = sbp_msg.tgd;
  ros_msg.c_rs = sbp_msg.c_rs;
  ros_msg.c_rc = sbp_msg.c_rc;
  ros_msg.c_uc = sbp_msg.c_uc;
  ros_msg.c_us = sbp_msg.c_us;
  ros_msg.c_ic = sbp_msg.c_ic;
  ros_msg.c_is = sbp_msg.c_is;
  ros_msg.dn = sbp_msg.dn;
  ros_msg.m0 = sbp_msg.m0;
  ros_msg.ecc = sbp_msg.ecc;
  ros_msg.sqrta = sbp_msg.sqrta;
  ros_msg.omega0 = sbp_msg.omega0;
  ros_msg.omegadot = sbp_msg.omegadot;
  ros_msg.w = sbp_msg.w;
  ros_msg.inc = sbp_msg.inc;
  ros_msg.inc_dot = sbp_msg.inc_dot;
  ros_msg.af0 = sbp_msg.af0;
  ros_msg.af1 = sbp_msg.af1;
  ros_msg.af2 = sbp_msg.af2;
  ros_msg.toc = convertSbpMsgToRosMsg(sbp_msg.toc);
  ros_msg.iode = sbp_msg.iode;
  ros_msg.iodc = sbp_msg.iodc;
  return ros_msg;
}

MsgEphemerisBds convertSbpMsgToRosMsg(const msg_ephemeris_bds_t& sbp_msg)
{
  MsgEphemerisBds ros_msg;
  ros_msg.common = convertSbpMsgToRosMsg(sbp_msg.common);
  ros_msg.tgd1 = sbp_msg.tgd1;
  ros_msg.tgd2 = sbp_msg.tgd2;
  ros_msg.c_rs = sbp_msg.c_rs;
  ros_msg.c_rc = sbp_msg.c_rc;
  ros_msg.c_uc = sbp_msg.c_uc;
  ros_msg.c_us = sbp_msg.c_us;
  ros_msg.c_ic = sbp_msg.c_ic;
  ros_msg.c_is = sbp_msg.c_is;
  ros_msg.dn = sbp_msg.dn;
  ros_msg.m0 = sbp_msg.m0;
  ros_msg.ecc = sbp_msg.ecc;
  ros_msg.sqrta = sbp_msg.sqrta;
  ros_msg.omega0 = sbp_msg.omega0;
  ros_msg.omegadot = sbp_msg.omegadot;
  ros_msg.w = sbp_msg.w;
  ros_msg.inc = sbp_msg.inc;
  ros_msg.inc_dot = sbp_msg.inc_dot;
  ros_msg.af0 = sbp_msg.af0;
  ros_msg.af1 = sbp_msg.af1;
  ros_msg.af2 = sbp_msg.af2;
  ros_msg.toc = convertSbpMsgToRosMsg(sbp_msg.toc);
  ros_msg.iode = sbp_msg.iode;
  ros_msg.iodc = sbp_msg.iodc;
  return ros_msg;
}

MsgEphemerisGal convertSbpMsgToRosMsg(const msg_ephemeris_gal_t& sbp_msg)
{
  MsgEphemerisGal ros_msg;
  ros_msg.common = convertSbpMsgToRosMsg(sbp_msg.common);
  ros_msg.bgd_e1e5a = sbp_msg.bgd_e1e5a;
  ros_msg.bgd_e1e5b = sbp_msg.bgd_e1e5b;
  ros_msg.c_rs = sbp_msg.c_rs;
  ros_msg.c_rc = sbp_msg.c_rc;
  ros_msg.c_uc = sbp_msg.c_uc;
  ros_msg.c_us = sbp_msg.c_us;
  ros_msg.c_ic = sbp_msg.c_ic;
  ros_msg.c_is = sbp_msg.c_is;
  ros_msg.dn = sbp_msg.dn;
  ros_msg.m0 = sbp_msg.m0;
  ros_msg.ecc = sbp_msg.ecc;
  ros_msg.sqrta = sbp_msg.sqrta;
  ros_msg.omega0 = sbp_msg.omega0;
  ros_msg.omegadot = sbp_msg.omegadot;
  ros_msg.w = sbp_msg.w;
  ros_msg.inc = sbp_msg.inc;
  ros_msg.inc_dot = sbp_msg.inc_dot;
  ros_msg.af0 = sbp_msg.af0;
  ros_msg.af1 = sbp_msg.af1;
  ros_msg.af2 = sbp_msg.af2;
  ros_msg.toc = convertSbpMsgToRosMsg(sbp_msg.toc);
  ros_msg.iode = sbp_msg.iode;
  ros_msg.iodc = sbp_msg.iodc;
  ros_msg.source = sbp_msg.source;
  return ros_msg;
}

MsgEphemerisSbas convertSbpMsgToRosMsg(const msg_ephemeris_sbas_t& sbp_msg)
{
  MsgEphemerisSbas ros_msg;
  ros_msg.common = convertSbpMsgToRosMsg(sbp_msg.common);
  for (auto msg : sbp_msg.pos)
    ros_msg.pos.push_back(msg);
  for (auto msg : sbp_msg.vel)
    ros_msg.vel.push_back(msg);
  for (auto msg : sbp_msg.acc)
    ros_msg.acc.push_back(msg);
  ros_msg.a_gf0 = sbp_msg.a_gf0;
  ros_msg.a_gf1 = sbp_msg.a_gf1;
  return ros_msg;
}

MsgEphemerisGlo convertSbpMsgToRosMsg(const msg_ephemeris_glo_t& sbp_msg)
{
  MsgEphemerisGlo ros_msg;
  ros_msg.common = convertSbpMsgToRosMsg(sbp_msg.common);
  ros_msg.gamma = sbp_msg.gamma;
  ros_msg.tau = sbp_msg.tau;
  ros_msg.d_tau = sbp_msg.d_tau;
  for (auto msg : sbp_msg.pos)
    ros_msg.pos.push_back(msg);
  for (auto msg : sbp_msg.vel)
    ros_msg.vel.push_back(msg);
  for (auto msg : sbp_msg.acc)
    ros_msg.acc.push_back(msg);
  ros_msg.fcn = sbp_msg.fcn;
  ros_msg.iod = sbp_msg.iod;
  return ros_msg;
}

MsgIono convertSbpMsgToRosMsg(const msg_iono_t& sbp_msg)
{
  MsgIono ros_msg;
  ros_msg.t_nmct = convertSbpMsgToRosMsg(sbp_msg.t_nmct);
  ros_msg.a0 = sbp_msg.a0;
  ros_msg.a1 = sbp_msg.a1;
  ros_msg.a2 = sbp_msg.a2;
  ros_msg.a3 = sbp_msg.a3;
  ros_msg.b0 = sbp_msg.b0;
  ros_msg.b1 = sbp_msg.b1;
  ros_msg.b2 = sbp_msg.b2;
  ros_msg.b3 = sbp_msg.b3;
  return ros_msg;
}

GnssCapb convertSbpMsgToRosMsg(const gnss_capb_t& sbp_msg)
{
  GnssCapb ros_msg;
  ros_msg.gps_active = sbp_msg.gps_active;
  ros_msg.gps_l2c = sbp_msg.gps_l2c;
  ros_msg.gps_l5 = sbp_msg.gps_l5;
  ros_msg.glo_active = sbp_msg.glo_active;
  ros_msg.glo_l2of = sbp_msg.glo_l2of;
  ros_msg.glo_l3 = sbp_msg.glo_l3;
  ros_msg.sbas_active = sbp_msg.sbas_active;
  ros_msg.sbas_l5 = sbp_msg.sbas_l5;
  ros_msg.bds_active = sbp_msg.bds_active;
  ros_msg.bds_d2nav = sbp_msg.bds_d2nav;
  ros_msg.bds_b2 = sbp_msg.bds_b2;
  ros_msg.bds_b2a = sbp_msg.bds_b2a;
  ros_msg.qzss_active = sbp_msg.qzss_active;
  ros_msg.gal_active = sbp_msg.gal_active;
  ros_msg.gal_e5 = sbp_msg.gal_e5;
  return ros_msg;
}

MsgGnssCapb convertSbpMsgToRosMsg(const msg_gnss_capb_t& sbp_msg)
{
  MsgGnssCapb ros_msg;
  ros_msg.t_nmct = convertSbpMsgToRosMsg(sbp_msg.t_nmct);
  ros_msg.gc = convertSbpMsgToRosMsg(sbp_msg.gc);
  return ros_msg;
}

MsgGroupDelay convertSbpMsgToRosMsg(const msg_group_delay_t& sbp_msg)
{
  MsgGroupDelay ros_msg;
  ros_msg.t_op = convertSbpMsgToRosMsg(sbp_msg.t_op);
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.valid = sbp_msg.valid;
  ros_msg.tgd = sbp_msg.tgd;
  ros_msg.isc_l1ca = sbp_msg.isc_l1ca;
  ros_msg.isc_l2c = sbp_msg.isc_l2c;
  return ros_msg;
}

AlmanacCommonContent convertSbpMsgToRosMsg(const almanac_common_content_t& sbp_msg)
{
  AlmanacCommonContent ros_msg;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.toa = convertSbpMsgToRosMsg(sbp_msg.toa);
  ros_msg.ura = sbp_msg.ura;
  ros_msg.fit_interval = sbp_msg.fit_interval;
  ros_msg.valid = sbp_msg.valid;
  ros_msg.health_bits = sbp_msg.health_bits;
  return ros_msg;
}

MsgAlmanacGps convertSbpMsgToRosMsg(const msg_almanac_gps_t& sbp_msg)
{
  MsgAlmanacGps ros_msg;
  ros_msg.common = convertSbpMsgToRosMsg(sbp_msg.common);
  ros_msg.m0 = sbp_msg.m0;
  ros_msg.ecc = sbp_msg.ecc;
  ros_msg.sqrta = sbp_msg.sqrta;
  ros_msg.omega0 = sbp_msg.omega0;
  ros_msg.omegadot = sbp_msg.omegadot;
  ros_msg.w = sbp_msg.w;
  ros_msg.inc = sbp_msg.inc;
  ros_msg.af0 = sbp_msg.af0;
  ros_msg.af1 = sbp_msg.af1;
  return ros_msg;
}

MsgAlmanacGlo convertSbpMsgToRosMsg(const msg_almanac_glo_t& sbp_msg)
{
  MsgAlmanacGlo ros_msg;
  ros_msg.common = convertSbpMsgToRosMsg(sbp_msg.common);
  ros_msg.lambda_na = sbp_msg.lambda_na;
  ros_msg.t_lambda_na = sbp_msg.t_lambda_na;
  ros_msg.i = sbp_msg.i;
  ros_msg.t = sbp_msg.t;
  ros_msg.t_dot = sbp_msg.t_dot;
  ros_msg.epsilon = sbp_msg.epsilon;
  ros_msg.omega = sbp_msg.omega;
  return ros_msg;
}

MsgGloBiases convertSbpMsgToRosMsg(const msg_glo_biases_t& sbp_msg)
{
  MsgGloBiases ros_msg;
  ros_msg.mask = sbp_msg.mask;
  ros_msg.l1ca_bias = sbp_msg.l1ca_bias;
  ros_msg.l1p_bias = sbp_msg.l1p_bias;
  ros_msg.l2ca_bias = sbp_msg.l2ca_bias;
  ros_msg.l2p_bias = sbp_msg.l2p_bias;
  return ros_msg;
}

SvAzEl convertSbpMsgToRosMsg(const sv_az_el_t& sbp_msg)
{
  SvAzEl ros_msg;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.az = sbp_msg.az;
  ros_msg.el = sbp_msg.el;
  return ros_msg;
}

MsgSvAzEl convertSbpMsgToRosMsg(const msg_sv_az_el_t& sbp_msg)
{
  MsgSvAzEl ros_msg;
  for (auto msg : sbp_msg.azel)
    ros_msg.azel.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MsgOsr convertSbpMsgToRosMsg(const msg_osr_t& sbp_msg)
{
  MsgOsr ros_msg;
  ros_msg.header = convertSbpMsgToRosMsg(sbp_msg.header);
  for (auto msg : sbp_msg.obs)
    ros_msg.obs.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MsgBaselineHeading convertSbpMsgToRosMsg(const msg_baseline_heading_t& sbp_msg)
{
  MsgBaselineHeading ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.heading = sbp_msg.heading;
  ros_msg.n_sats = sbp_msg.n_sats;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgOrientQuat convertSbpMsgToRosMsg(const msg_orient_quat_t& sbp_msg)
{
  MsgOrientQuat ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.w = sbp_msg.w;
  ros_msg.x = sbp_msg.x;
  ros_msg.y = sbp_msg.y;
  ros_msg.z = sbp_msg.z;
  ros_msg.w_accuracy = sbp_msg.w_accuracy;
  ros_msg.x_accuracy = sbp_msg.x_accuracy;
  ros_msg.y_accuracy = sbp_msg.y_accuracy;
  ros_msg.z_accuracy = sbp_msg.z_accuracy;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgOrientEuler convertSbpMsgToRosMsg(const msg_orient_euler_t& sbp_msg)
{
  MsgOrientEuler ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.roll = sbp_msg.roll;
  ros_msg.pitch = sbp_msg.pitch;
  ros_msg.yaw = sbp_msg.yaw;
  ros_msg.roll_accuracy = sbp_msg.roll_accuracy;
  ros_msg.pitch_accuracy = sbp_msg.pitch_accuracy;
  ros_msg.yaw_accuracy = sbp_msg.yaw_accuracy;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgAngularRate convertSbpMsgToRosMsg(const msg_angular_rate_t& sbp_msg)
{
  MsgAngularRate ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.x = sbp_msg.x;
  ros_msg.y = sbp_msg.y;
  ros_msg.z = sbp_msg.z;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgReset convertSbpMsgToRosMsg(const msg_reset_t& sbp_msg)
{
  MsgReset ros_msg;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgResetFilters convertSbpMsgToRosMsg(const msg_reset_filters_t& sbp_msg)
{
  MsgResetFilters ros_msg;
  ros_msg.filter = sbp_msg.filter;
  return ros_msg;
}

MsgThreadState convertSbpMsgToRosMsg(const msg_thread_state_t& sbp_msg)
{
  MsgThreadState ros_msg;
  ros_msg.name = sbp_msg.name;
  ros_msg.cpu = sbp_msg.cpu;
  ros_msg.stack_free = sbp_msg.stack_free;
  return ros_msg;
}

UartChannel convertSbpMsgToRosMsg(const uart_channel_t& sbp_msg)
{
  UartChannel ros_msg;
  ros_msg.tx_throughput = sbp_msg.tx_throughput;
  ros_msg.rx_throughput = sbp_msg.rx_throughput;
  ros_msg.crc_error_count = sbp_msg.crc_error_count;
  ros_msg.io_error_count = sbp_msg.io_error_count;
  ros_msg.tx_buffer_level = sbp_msg.tx_buffer_level;
  ros_msg.rx_buffer_level = sbp_msg.rx_buffer_level;
  return ros_msg;
}

Period convertSbpMsgToRosMsg(const period_t& sbp_msg)
{
  Period ros_msg;
  ros_msg.avg = sbp_msg.avg;
  ros_msg.pmin = sbp_msg.pmin;
  ros_msg.pmax = sbp_msg.pmax;
  ros_msg.current = sbp_msg.current;
  return ros_msg;
}

Latency convertSbpMsgToRosMsg(const latency_t& sbp_msg)
{
  Latency ros_msg;
  ros_msg.avg = sbp_msg.avg;
  ros_msg.lmin = sbp_msg.lmin;
  ros_msg.lmax = sbp_msg.lmax;
  ros_msg.current = sbp_msg.current;
  return ros_msg;
}

MsgUartState convertSbpMsgToRosMsg(const msg_uart_state_t& sbp_msg)
{
  MsgUartState ros_msg;
  ros_msg.uart_a = convertSbpMsgToRosMsg(sbp_msg.uart_a);
  ros_msg.uart_b = convertSbpMsgToRosMsg(sbp_msg.uart_b);
  ros_msg.uart_ftdi = convertSbpMsgToRosMsg(sbp_msg.uart_ftdi);
  ros_msg.latency = convertSbpMsgToRosMsg(sbp_msg.latency);
  ros_msg.obs_period = convertSbpMsgToRosMsg(sbp_msg.obs_period);
  return ros_msg;
}

MsgIarState convertSbpMsgToRosMsg(const msg_iar_state_t& sbp_msg)
{
  MsgIarState ros_msg;
  ros_msg.num_hyps = sbp_msg.num_hyps;
  return ros_msg;
}

MsgMaskSatellite convertSbpMsgToRosMsg(const msg_mask_satellite_t& sbp_msg)
{
  MsgMaskSatellite ros_msg;
  ros_msg.mask = sbp_msg.mask;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  return ros_msg;
}

MsgDeviceMonitor convertSbpMsgToRosMsg(const msg_device_monitor_t& sbp_msg)
{
  MsgDeviceMonitor ros_msg;
  ros_msg.dev_vin = sbp_msg.dev_vin;
  ros_msg.cpu_vint = sbp_msg.cpu_vint;
  ros_msg.cpu_vaux = sbp_msg.cpu_vaux;
  ros_msg.cpu_temperature = sbp_msg.cpu_temperature;
  ros_msg.fe_temperature = sbp_msg.fe_temperature;
  return ros_msg;
}

MsgCommandReq convertSbpMsgToRosMsg(const msg_command_req_t& sbp_msg)
{
  MsgCommandReq ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  ros_msg.command = sbp_msg.command;
  return ros_msg;
}

MsgCommandResp convertSbpMsgToRosMsg(const msg_command_resp_t& sbp_msg)
{
  MsgCommandResp ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  ros_msg.code = sbp_msg.code;
  return ros_msg;
}

MsgCommandOutput convertSbpMsgToRosMsg(const msg_command_output_t& sbp_msg)
{
  MsgCommandOutput ros_msg;
  ros_msg.sequence = sbp_msg.sequence;
  ros_msg.line = sbp_msg.line;
  return ros_msg;
}

MsgNetworkStateResp convertSbpMsgToRosMsg(const msg_network_state_resp_t& sbp_msg)
{
  MsgNetworkStateResp ros_msg;
  for (auto msg : sbp_msg.ipv4_address)
    ros_msg.ipv4_address.push_back(msg);
  ros_msg.ipv4_mask_size = sbp_msg.ipv4_mask_size;
  for (auto msg : sbp_msg.ipv6_address)
    ros_msg.ipv6_address.push_back(msg);
  ros_msg.ipv6_mask_size = sbp_msg.ipv6_mask_size;
  ros_msg.rx_bytes = sbp_msg.rx_bytes;
  ros_msg.tx_bytes = sbp_msg.tx_bytes;
  ros_msg.interface_name = sbp_msg.interface_name;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

NetworkUsage convertSbpMsgToRosMsg(const network_usage_t& sbp_msg)
{
  NetworkUsage ros_msg;
  ros_msg.duration = sbp_msg.duration;
  ros_msg.total_bytes = sbp_msg.total_bytes;
  ros_msg.rx_bytes = sbp_msg.rx_bytes;
  ros_msg.tx_bytes = sbp_msg.tx_bytes;
  ros_msg.interface_name = sbp_msg.interface_name;
  return ros_msg;
}

MsgNetworkBandwidthUsage convertSbpMsgToRosMsg(const msg_network_bandwidth_usage_t& sbp_msg)
{
  MsgNetworkBandwidthUsage ros_msg;
  for (auto msg : sbp_msg.interfaces)
    ros_msg.interfaces.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MsgCellModemStatus convertSbpMsgToRosMsg(const msg_cell_modem_status_t& sbp_msg)
{
  MsgCellModemStatus ros_msg;
  ros_msg.signal_strength = sbp_msg.signal_strength;
  ros_msg.signal_error_rate = sbp_msg.signal_error_rate;
  return ros_msg;
}

MsgSpecan convertSbpMsgToRosMsg(const msg_specan_t& sbp_msg)
{
  MsgSpecan ros_msg;
  ros_msg.channel_tag = sbp_msg.channel_tag;
  ros_msg.t = convertSbpMsgToRosMsg(sbp_msg.t);
  ros_msg.freq_ref = sbp_msg.freq_ref;
  ros_msg.freq_step = sbp_msg.freq_step;
  ros_msg.amplitude_ref = sbp_msg.amplitude_ref;
  ros_msg.amplitude_unit = sbp_msg.amplitude_unit;
  for (auto msg : sbp_msg.amplitude_value)
    ros_msg.amplitude_value.push_back(msg);
  return ros_msg;
}

MsgFrontEndGain convertSbpMsgToRosMsg(const msg_front_end_gain_t& sbp_msg)
{
  MsgFrontEndGain ros_msg;
  for (auto msg : sbp_msg.rf_gain)
    ros_msg.rf_gain.push_back(msg);
  for (auto msg : sbp_msg.if_gain)
    ros_msg.if_gain.push_back(msg);
  return ros_msg;
}

MsgSbasRaw convertSbpMsgToRosMsg(const msg_sbas_raw_t& sbp_msg)
{
  MsgSbasRaw ros_msg;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.tow = sbp_msg.tow;
  ros_msg.message_type = sbp_msg.message_type;
  for (auto msg : sbp_msg.data)
    ros_msg.data.push_back(msg);
  return ros_msg;
}

MsgSettingsWrite convertSbpMsgToRosMsg(const msg_settings_write_t& sbp_msg)
{
  MsgSettingsWrite ros_msg;
  ros_msg.setting = sbp_msg.setting;
  return ros_msg;
}

MsgSettingsWriteResp convertSbpMsgToRosMsg(const msg_settings_write_resp_t& sbp_msg)
{
  MsgSettingsWriteResp ros_msg;
  ros_msg.status = sbp_msg.status;
  ros_msg.setting = sbp_msg.setting;
  return ros_msg;
}

MsgSettingsReadReq convertSbpMsgToRosMsg(const msg_settings_read_req_t& sbp_msg)
{
  MsgSettingsReadReq ros_msg;
  ros_msg.setting = sbp_msg.setting;
  return ros_msg;
}

MsgSettingsReadResp convertSbpMsgToRosMsg(const msg_settings_read_resp_t& sbp_msg)
{
  MsgSettingsReadResp ros_msg;
  ros_msg.setting = sbp_msg.setting;
  return ros_msg;
}

MsgSettingsReadByIndexReq convertSbpMsgToRosMsg(const msg_settings_read_by_index_req_t& sbp_msg)
{
  MsgSettingsReadByIndexReq ros_msg;
  ros_msg.index = sbp_msg.index;
  return ros_msg;
}

MsgSettingsReadByIndexResp convertSbpMsgToRosMsg(const msg_settings_read_by_index_resp_t& sbp_msg)
{
  MsgSettingsReadByIndexResp ros_msg;
  ros_msg.index = sbp_msg.index;
  ros_msg.setting = sbp_msg.setting;
  return ros_msg;
}

MsgSettingsRegister convertSbpMsgToRosMsg(const msg_settings_register_t& sbp_msg)
{
  MsgSettingsRegister ros_msg;
  ros_msg.setting = sbp_msg.setting;
  return ros_msg;
}

MsgSettingsRegisterResp convertSbpMsgToRosMsg(const msg_settings_register_resp_t& sbp_msg)
{
  MsgSettingsRegisterResp ros_msg;
  ros_msg.status = sbp_msg.status;
  ros_msg.setting = sbp_msg.setting;
  return ros_msg;
}

CodeBiasesContent convertSbpMsgToRosMsg(const code_biases_content_t& sbp_msg)
{
  CodeBiasesContent ros_msg;
  ros_msg.code = sbp_msg.code;
  ros_msg.value = sbp_msg.value;
  return ros_msg;
}

PhaseBiasesContent convertSbpMsgToRosMsg(const phase_biases_content_t& sbp_msg)
{
  PhaseBiasesContent ros_msg;
  ros_msg.code = sbp_msg.code;
  ros_msg.integer_indicator = sbp_msg.integer_indicator;
  ros_msg.widelane_integer_indicator = sbp_msg.widelane_integer_indicator;
  ros_msg.discontinuity_counter = sbp_msg.discontinuity_counter;
  ros_msg.bias = sbp_msg.bias;
  return ros_msg;
}

StecHeader convertSbpMsgToRosMsg(const stec_header_t& sbp_msg)
{
  StecHeader ros_msg;
  ros_msg.time = convertSbpMsgToRosMsg(sbp_msg.time);
  ros_msg.num_msgs = sbp_msg.num_msgs;
  ros_msg.seq_num = sbp_msg.seq_num;
  ros_msg.ssr_update_interval = sbp_msg.ssr_update_interval;
  ros_msg.update_interval = sbp_msg.update_interval;
  ros_msg.iod_ssr = sbp_msg.iod_ssr;
  return ros_msg;
}

GriddedCorrectionHeader convertSbpMsgToRosMsg(const gridded_correction_header_t& sbp_msg)
{
  GriddedCorrectionHeader ros_msg;
  ros_msg.time = convertSbpMsgToRosMsg(sbp_msg.time);
  ros_msg.num_msgs = sbp_msg.num_msgs;
  ros_msg.seq_num = sbp_msg.seq_num;
  ros_msg.update_interval = sbp_msg.update_interval;
  ros_msg.iod_ssr = sbp_msg.iod_ssr;
  ros_msg.tropo_quality_indicator = sbp_msg.tropo_quality_indicator;
  return ros_msg;
}

StecSatElement convertSbpMsgToRosMsg(const stec_sat_element_t& sbp_msg)
{
  StecSatElement ros_msg;
  ros_msg.sv_id = convertSbpMsgToRosMsg(sbp_msg.sv_id);
  ros_msg.stec_quality_indicator = sbp_msg.stec_quality_indicator;
  for (auto msg : sbp_msg.stec_coeff)
    ros_msg.stec_coeff.push_back(msg);
  return ros_msg;
}

TroposphericDelayCorrection convertSbpMsgToRosMsg(const tropospheric_delay_correction_t& sbp_msg)
{
  TroposphericDelayCorrection ros_msg;
  ros_msg.hydro = sbp_msg.hydro;
  ros_msg.wet = sbp_msg.wet;
  return ros_msg;
}

StecResidual convertSbpMsgToRosMsg(const stec_residual_t& sbp_msg)
{
  StecResidual ros_msg;
  ros_msg.sv_id = convertSbpMsgToRosMsg(sbp_msg.sv_id);
  ros_msg.residual = sbp_msg.residual;
  return ros_msg;
}

GridElement convertSbpMsgToRosMsg(const grid_element_t& sbp_msg)
{
  GridElement ros_msg;
  ros_msg.index = sbp_msg.index;
  ros_msg.tropo_delay_correction = convertSbpMsgToRosMsg(sbp_msg.tropo_delay_correction);
  for (auto msg : sbp_msg.stec_residuals)
    ros_msg.stec_residuals.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

GridDefinitionHeader convertSbpMsgToRosMsg(const grid_definition_header_t& sbp_msg)
{
  GridDefinitionHeader ros_msg;
  ros_msg.region_size_inverse = sbp_msg.region_size_inverse;
  ros_msg.area_width = sbp_msg.area_width;
  ros_msg.lat_nw_corner_enc = sbp_msg.lat_nw_corner_enc;
  ros_msg.lon_nw_corner_enc = sbp_msg.lon_nw_corner_enc;
  ros_msg.num_msgs = sbp_msg.num_msgs;
  ros_msg.seq_num = sbp_msg.seq_num;
  return ros_msg;
}

MsgSsrOrbitClock convertSbpMsgToRosMsg(const msg_ssr_orbit_clock_t& sbp_msg)
{
  MsgSsrOrbitClock ros_msg;
  ros_msg.time = convertSbpMsgToRosMsg(sbp_msg.time);
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.update_interval = sbp_msg.update_interval;
  ros_msg.iod_ssr = sbp_msg.iod_ssr;
  ros_msg.iod = sbp_msg.iod;
  ros_msg.radial = sbp_msg.radial;
  ros_msg.along = sbp_msg.along;
  ros_msg.cross = sbp_msg.cross;
  ros_msg.dot_radial = sbp_msg.dot_radial;
  ros_msg.dot_along = sbp_msg.dot_along;
  ros_msg.dot_cross = sbp_msg.dot_cross;
  ros_msg.c0 = sbp_msg.c0;
  ros_msg.c1 = sbp_msg.c1;
  ros_msg.c2 = sbp_msg.c2;
  return ros_msg;
}

MsgSsrCodeBiases convertSbpMsgToRosMsg(const msg_ssr_code_biases_t& sbp_msg)
{
  MsgSsrCodeBiases ros_msg;
  ros_msg.time = convertSbpMsgToRosMsg(sbp_msg.time);
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.update_interval = sbp_msg.update_interval;
  ros_msg.iod_ssr = sbp_msg.iod_ssr;
  for (auto msg : sbp_msg.biases)
    ros_msg.biases.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MsgSsrPhaseBiases convertSbpMsgToRosMsg(const msg_ssr_phase_biases_t& sbp_msg)
{
  MsgSsrPhaseBiases ros_msg;
  ros_msg.time = convertSbpMsgToRosMsg(sbp_msg.time);
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.update_interval = sbp_msg.update_interval;
  ros_msg.iod_ssr = sbp_msg.iod_ssr;
  ros_msg.dispersive_bias = sbp_msg.dispersive_bias;
  ros_msg.mw_consistency = sbp_msg.mw_consistency;
  ros_msg.yaw = sbp_msg.yaw;
  ros_msg.yaw_rate = sbp_msg.yaw_rate;
  for (auto msg : sbp_msg.biases)
    ros_msg.biases.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MsgSsrStecCorrection convertSbpMsgToRosMsg(const msg_ssr_stec_correction_t& sbp_msg)
{
  MsgSsrStecCorrection ros_msg;
  ros_msg.header = convertSbpMsgToRosMsg(sbp_msg.header);
  for (auto msg : sbp_msg.stec_sat_list)
    ros_msg.stec_sat_list.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MsgSsrGriddedCorrection convertSbpMsgToRosMsg(const msg_ssr_gridded_correction_t& sbp_msg)
{
  MsgSsrGriddedCorrection ros_msg;
  ros_msg.header = convertSbpMsgToRosMsg(sbp_msg.header);
  ros_msg.element = convertSbpMsgToRosMsg(sbp_msg.element);
  return ros_msg;
}

MsgSsrGridDefinition convertSbpMsgToRosMsg(const msg_ssr_grid_definition_t& sbp_msg)
{
  MsgSsrGridDefinition ros_msg;
  ros_msg.header = convertSbpMsgToRosMsg(sbp_msg.header);
  for (auto msg : sbp_msg.rle_list)
    ros_msg.rle_list.push_back(msg);
  return ros_msg;
}

MsgStartup convertSbpMsgToRosMsg(const msg_startup_t& sbp_msg)
{
  MsgStartup ros_msg;
  ros_msg.cause = sbp_msg.cause;
  ros_msg.startup_type = sbp_msg.startup_type;
  return ros_msg;
}

MsgDgnssStatus convertSbpMsgToRosMsg(const msg_dgnss_status_t& sbp_msg)
{
  MsgDgnssStatus ros_msg;
  ros_msg.flags = sbp_msg.flags;
  ros_msg.latency = sbp_msg.latency;
  ros_msg.num_signals = sbp_msg.num_signals;
  ros_msg.source = sbp_msg.source;
  return ros_msg;
}

MsgHeartbeat convertSbpMsgToRosMsg(const msg_heartbeat_t& sbp_msg)
{
  MsgHeartbeat ros_msg;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgInsStatus convertSbpMsgToRosMsg(const msg_ins_status_t& sbp_msg)
{
  MsgInsStatus ros_msg;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}

MsgCsacTelemetry convertSbpMsgToRosMsg(const msg_csac_telemetry_t& sbp_msg)
{
  MsgCsacTelemetry ros_msg;
  ros_msg.id = sbp_msg.id;
  ros_msg.telemetry = sbp_msg.telemetry;
  return ros_msg;
}

MsgCsacTelemetryLabels convertSbpMsgToRosMsg(const msg_csac_telemetry_labels_t& sbp_msg)
{
  MsgCsacTelemetryLabels ros_msg;
  ros_msg.id = sbp_msg.id;
  ros_msg.telemetry_labels = sbp_msg.telemetry_labels;
  return ros_msg;
}

TrackingChannelState convertSbpMsgToRosMsg(const tracking_channel_state_t& sbp_msg)
{
  TrackingChannelState ros_msg;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  ros_msg.fcn = sbp_msg.fcn;
  ros_msg.cn0 = sbp_msg.cn0;
  return ros_msg;
}

MsgTrackingState convertSbpMsgToRosMsg(const msg_tracking_state_t& sbp_msg)
{
  MsgTrackingState ros_msg;
  for (auto msg : sbp_msg.states)
    ros_msg.states.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MeasurementState convertSbpMsgToRosMsg(const measurement_state_t& sbp_msg)
{
  MeasurementState ros_msg;
  ros_msg.mesid = convertSbpMsgToRosMsg(sbp_msg.mesid);
  ros_msg.cn0 = sbp_msg.cn0;
  return ros_msg;
}

MsgMeasurementState convertSbpMsgToRosMsg(const msg_measurement_state_t& sbp_msg)
{
  MsgMeasurementState ros_msg;
  for (auto msg : sbp_msg.states)
    ros_msg.states.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

TrackingChannelCorrelation convertSbpMsgToRosMsg(const tracking_channel_correlation_t& sbp_msg)
{
  TrackingChannelCorrelation ros_msg;
  ros_msg.I = sbp_msg.I;
  ros_msg.Q = sbp_msg.Q;
  return ros_msg;
}

MsgTrackingIq convertSbpMsgToRosMsg(const msg_tracking_iq_t& sbp_msg)
{
  MsgTrackingIq ros_msg;
  ros_msg.channel = sbp_msg.channel;
  ros_msg.sid = convertSbpMsgToRosMsg(sbp_msg.sid);
  for (auto msg : sbp_msg.corrs)
    ros_msg.corrs.push_back(convertSbpMsgToRosMsg(msg));
  return ros_msg;
}

MsgUserData convertSbpMsgToRosMsg(const msg_user_data_t& sbp_msg)
{
  MsgUserData ros_msg;
  for (auto msg : sbp_msg.contents)
    ros_msg.contents.push_back(msg);
  return ros_msg;
}

MsgOdometry convertSbpMsgToRosMsg(const msg_odometry_t& sbp_msg)
{
  MsgOdometry ros_msg;
  ros_msg.tow = sbp_msg.tow;
  ros_msg.velocity = sbp_msg.velocity;
  ros_msg.flags = sbp_msg.flags;
  return ros_msg;
}
} // namespace piksi_multi_msgs