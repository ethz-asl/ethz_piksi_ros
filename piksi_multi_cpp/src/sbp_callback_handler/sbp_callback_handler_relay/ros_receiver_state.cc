#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_receiver_state.h"

#include <libsbp_ros_msgs/conversion.h>
#include <functional>

namespace piksi_multi_cpp {

RosReceiverState::RosReceiverState(const ros::NodeHandle& nh,
                                   const std::shared_ptr<sbp_state_t>& state,
                                   const RosTimeHandler::Ptr& ros_time_handler)
    : SBPCallbackHandlerRelay<msg_measurement_state_t, ReceiverState>(
          nh, SBP_MSG_MEASUREMENT_STATE, state, "ros/receiver_state"),
      ros_time_handler_(ros_time_handler),
      pos_ecef_handler_{
          std::bind(&RosReceiverState::callbackToGnssSolution<msg_pos_ecef_t>,
                    this, std::placeholders::_1, std::placeholders::_2),
          SBP_MSG_POS_ECEF, state},
      pos_ecef_cov_handler_{
          std::bind(
              &RosReceiverState::callbackToGnssSolution<msg_pos_ecef_cov_t>,
              this, std::placeholders::_1, std::placeholders::_2),
          SBP_MSG_POS_ECEF_COV, state},
      pos_llh_handler_{
          std::bind(&RosReceiverState::callbackToGnssSolution<msg_pos_llh_t>,
                    this, std::placeholders::_1, std::placeholders::_2),
          SBP_MSG_POS_LLH, state},
      pos_llh_cov_handler_{
          std::bind(
              &RosReceiverState::callbackToGnssSolution<msg_pos_llh_cov_t>,
              this, std::placeholders::_1, std::placeholders::_2),
          SBP_MSG_POS_LLH_COV, state},
      heartbeat_handler_{
          std::bind(&RosReceiverState::callbackToHeartbeat, this,
                    std::placeholders::_1, std::placeholders::_2),
          SBP_MSG_HEARTBEAT, state} {
  // Initialize receiver
  receiver_state_.rtk_mode_fix = false;
  receiver_state_.system_error = ReceiverState::STATUS_UNKNOWN;
  receiver_state_.io_error = ReceiverState::STATUS_UNKNOWN;
  receiver_state_.swift_nap_error = ReceiverState::STATUS_UNKNOWN;
  receiver_state_.external_antenna_short = ReceiverState::STATUS_UNKNOWN;
  receiver_state_.external_antenna_present = ReceiverState::STATUS_UNKNOWN;
  receiver_state_.fix_mode = ReceiverState::STR_FIX_MODE_UNKNOWN;
  receiver_state_.utc_time_ready = false;
  resetMeasurementState();
}

uint16_t RosReceiverState::getNavSatServiceStatus() const {
  uint16_t service_status = 0;
  if (receiver_state_.num_gps_sat > 0) service_status += (1 << 0);
  if (receiver_state_.num_glonass_sat > 0) service_status += (1 << 1);
  if (receiver_state_.num_bds_sat > 0) service_status += (1 << 2);
  if (receiver_state_.num_gal_sat > 0) service_status += (1 << 3);
  return service_status;
}

void RosReceiverState::callbackToHeartbeat(const msg_heartbeat_t& msg,
                                           const uint8_t len) {
  receiver_state_.system_error = (msg.flags >> 0) & 0x1;
  receiver_state_.io_error = (msg.flags >> 1) & 0x1;
  receiver_state_.swift_nap_error = (msg.flags >> 2) & 0x1;
  receiver_state_.external_antenna_short = (msg.flags >> 30) & 0x1;
  receiver_state_.external_antenna_present = (msg.flags >> 31) & 0x1;
}

void RosReceiverState::resetMeasurementState() {
  receiver_state_.num_sat = 0;
  receiver_state_.sat.clear();
  receiver_state_.cn0.clear();
  receiver_state_.num_gps_sat = 0;
  receiver_state_.cn0_gps.clear();
  receiver_state_.num_sbas_sat = 0;
  receiver_state_.cn0_sbas.clear();
  receiver_state_.num_glonass_sat = 0;
  receiver_state_.cn0_glonass.clear();
  receiver_state_.num_bds_sat = 0;
  receiver_state_.cn0_bds.clear();
  receiver_state_.num_gal_sat = 0;
  receiver_state_.cn0_gal.clear();
}

bool RosReceiverState::convertSbpToRos(const msg_measurement_state_t& sbp_msg,
                                       const uint8_t len,
                                       ReceiverState* ros_msg) {
  libsbp_ros_msgs::MsgMeasurementState meas_states =
      libsbp_ros_msgs::convertSbpMsgToRosMsg(sbp_msg, len);

  // Collect satellite information.
  resetMeasurementState();
  for (const auto& meas : meas_states.states) {
    if (meas.cn0 == 0) continue;
    switch (meas.mesid.code) {
      case SidCodeValues::kGpsL1CA:
      case SidCodeValues::kGpsL2CM:
      case SidCodeValues::kGpsL1P:
      case SidCodeValues::kGpsL2P:
        receiver_state_.num_gps_sat++;
        receiver_state_.cn0_gps.push_back(meas.cn0);
        receiver_state_.num_sat++;
        receiver_state_.sat.push_back(meas.mesid.sat);
        receiver_state_.cn0.push_back(meas.cn0);
        break;
      case SidCodeValues::kSbasL1CA:
        receiver_state_.num_sbas_sat++;
        receiver_state_.cn0_sbas.push_back(meas.cn0);
        receiver_state_.num_sat++;
        receiver_state_.sat.push_back(meas.mesid.sat);
        receiver_state_.cn0.push_back(meas.cn0);
        break;
      case SidCodeValues::kGloL1CA:
      case SidCodeValues::kGloL2CA:
        receiver_state_.num_glonass_sat++;
        receiver_state_.cn0_glonass.push_back(meas.cn0);
        receiver_state_.num_sat++;
        receiver_state_.sat.push_back(meas.mesid.sat);
        receiver_state_.cn0.push_back(meas.cn0);
        break;
      case SidCodeValues::kBds2B1:
      case SidCodeValues::kBds2B2:
        receiver_state_.num_bds_sat++;
        receiver_state_.cn0_bds.push_back(meas.cn0);
        receiver_state_.num_sat++;
        receiver_state_.sat.push_back(meas.mesid.sat);
        receiver_state_.cn0.push_back(meas.cn0);
        break;
      case SidCodeValues::kGalE1B:
      case SidCodeValues::kGalE7I:
        receiver_state_.num_gal_sat++;
        receiver_state_.cn0_gal.push_back(meas.cn0);
        receiver_state_.num_sat++;
        receiver_state_.sat.push_back(meas.mesid.sat);
        receiver_state_.cn0.push_back(meas.cn0);
        break;
    }
  }

  receiver_state_.utc_time_ready = ros_time_handler_->utcTimeReady();
  receiver_state_.header.stamp = ros::Time::now();
  *ros_msg = receiver_state_;
  return true;
}

}  // namespace piksi_multi_cpp
