#include <ros/console.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay_ext_events.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay_imu.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay_logging.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay_mag.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay_system.h"

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

namespace piksi_multi_cpp {

SBPCallbackHandler::Ptr
SBPCallbackHandlerFactory::createRelayCallbackBySBPMsgType(
    const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
    const std::shared_ptr<sbp_state_t>& state) {
  switch (sbp_msg_type) {
    case SBP_MSG_EXT_EVENT:
      return SBPCallbackHandler::Ptr(
          new SBPCallbackHandlerRelayExtEvent(nh, state));
    case SBP_MSG_IMU_RAW:
      return SBPCallbackHandler::Ptr(
          new SBPCallbackHandlerRelayImuRaw(nh, state));
    case SBP_MSG_IMU_AUX:
      return SBPCallbackHandler::Ptr(
          new SBPCallbackHandlerRelayImuAux(nh, state));
    case SBP_MSG_LOG:
      return SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayLog(nh, state));
    case SBP_MSG_FWD:
      return SBPCallbackHandler::Ptr(new SBPCallbackHandlerRelayFwd(nh, state));
    case SBP_MSG_MAG_RAW:
      return SBPCallbackHandler::Ptr(
          new SBPCallbackHandlerRelayMagRaw(nh, state));
    case SBP_MSG_HEARTBEAT:
      return SBPCallbackHandler::Ptr(
          new SBPCallbackHandlerRelayHeartbeat(nh, state));
    default:
      // TODO(rikba): Implement all other callbacks.
      ROS_WARN("Message type %u not implemented.", sbp_msg_type);
      return nullptr;
  }
}

// Factory method to create all implemented SBP message relays.
std::vector<SBPCallbackHandler::Ptr>
SBPCallbackHandlerFactory::createAllSBPMessageRelays(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  std::vector<SBPCallbackHandler::Ptr> cbs;
  // Ext Event
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_EXT_EVENT, state));
  // Imu
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_IMU_RAW, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_IMU_AUX, state));
  // Logging
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_LOG, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_FWD, state));
  // Mag
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_MAG_RAW, state));
  // Navigation
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_GPS_TIME, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_UTC_TIME, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_DOPS, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_POS_ECEF, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_POS_ECEF_COV, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_POS_LLH, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_POS_LLH_COV, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_BASELINE_ECEF, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_BASELINE_NED, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_VEL_ECEF, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_VEL_ECEF_COV, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_VEL_NED, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_VEL_NED_COV, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_VEL_BODY, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_AGE_CORRECTIONS, state));
  // Observation
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_OBS, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_BASE_POS_LLH, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_BASE_POS_ECEF, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_EPHEMERIS_GPS, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_EPHEMERIS_QZSS, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_EPHEMERIS_BDS, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_EPHEMERIS_GAL, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_EPHEMERIS_SBAS, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_EPHEMERIS_GLO, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_IONO, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(
      nh, SBP_MSG_SV_CONFIGURATION_GPS_DEP, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_GNSS_CAPB, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_GROUP_DELAY, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_ALMANAC_GPS, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_ALMANAC_GLO, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_GLO_BIASES, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_SV_AZ_EL, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_OSR, state));
  // System
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_STARTUP, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_DGNSS_STATUS, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_HEARTBEAT, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_INS_STATUS, state));
  // Acquisition
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_ACQ_RESULT, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_ACQ_SV_PROFILE, state));
  // Linux
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_LINUX_CPU_STATE, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_LINUX_MEM_STATE, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_LINUX_SYS_STATE, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(
      nh, SBP_MSG_LINUX_PROCESS_SOCKET_COUNTS, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(
      nh, SBP_MSG_LINUX_PROCESS_SOCKET_QUEUES, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_LINUX_SOCKET_USAGE, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(
      nh, SBP_MSG_LINUX_PROCESS_FD_COUNT, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(
      nh, SBP_MSG_LINUX_PROCESS_FD_SUMMARY, state));
  // Orientation
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_BASELINE_HEADING, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_ORIENT_QUAT, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_ORIENT_EULER, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_ANGULAR_RATE, state));
  // Piksi
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_THREAD_STATE, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_UART_STATE, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_IAR_STATE, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_DEVICE_MONITOR, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_COMMAND_RESP, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_COMMAND_OUTPUT, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_NETWORK_STATE_RESP, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(
      nh, SBP_MSG_NETWORK_BANDWIDTH_USAGE, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_CELL_MODEM_STATUS, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_SPECAN, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_FRONT_END_GAIN, state));
  // Sbas
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_SBAS_RAW, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_SSR_ORBIT_CLOCK, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_SSR_CODE_BIASES, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_SSR_PHASE_BIASES, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_SSR_STEC_CORRECTION, state));
  cbs.push_back(createRelayCallbackBySBPMsgType(
      nh, SBP_MSG_SSR_GRIDDED_CORRECTION, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_SSR_GRID_DEFINITION, state));
  // Tracking
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_TRACKING_STATE, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_MEASUREMENT_STATE, state));
  cbs.push_back(
      createRelayCallbackBySBPMsgType(nh, SBP_MSG_TRACKING_IQ, state));
  // User
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_USER_DATA, state));
  // Vehicle
  cbs.push_back(createRelayCallbackBySBPMsgType(nh, SBP_MSG_ODOMETRY, state));

  // Remove all invalid (nullptr) callbacks.
  cbs.erase(std::remove_if(
      cbs.begin(), cbs.end(),
      [](const SBPCallbackHandler::Ptr& cb) { return cb.get() == nullptr; }));

  return cbs;
}

}  // namespace piksi_multi_cpp
