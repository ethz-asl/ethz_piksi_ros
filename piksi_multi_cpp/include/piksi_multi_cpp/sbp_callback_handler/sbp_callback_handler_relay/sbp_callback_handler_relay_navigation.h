#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_NAVIGATION_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_NAVIGATION_H_

#include <libsbp/navigation.h>
#include <piksi_multi_msgs/AgeCorrections.h>
#include <piksi_multi_msgs/BaselineEcef.h>
#include <piksi_multi_msgs/BaselineNed.h>
#include <piksi_multi_msgs/Dops.h>
#include <piksi_multi_msgs/GpsTime.h>
#include <piksi_multi_msgs/PosEcef.h>
#include <piksi_multi_msgs/PosEcefCov.h>
#include <piksi_multi_msgs/PosLlh.h>
#include <piksi_multi_msgs/PosLlhCov.h>
#include <piksi_multi_msgs/UtcTime.h>
#include <piksi_multi_msgs/VelBody.h>
#include <piksi_multi_msgs/VelEcef.h>
#include <piksi_multi_msgs/VelEcefCov.h>
#include <piksi_multi_msgs/VelNed.h>
#include <piksi_multi_msgs/VelNedCov.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayGpsTime
    : public SBPCallbackHandlerRelay<msg_gps_time_t,
                                     piksi_multi_msgs::GpsTime> {
 public:
  inline SBPCallbackHandlerRelayGpsTime(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_GPS_TIME, state, "gps_time") {}
};

class SBPCallbackHandlerRelayUtcTime
    : public SBPCallbackHandlerRelay<msg_utc_time_t,
                                     piksi_multi_msgs::UtcTime> {
 public:
  inline SBPCallbackHandlerRelayUtcTime(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_UTC_TIME, state, "utc_time") {}
};

class SBPCallbackHandlerRelayDops
    : public SBPCallbackHandlerRelay<msg_dops_t, piksi_multi_msgs::Dops> {
 public:
  inline SBPCallbackHandlerRelayDops(const ros::NodeHandle& nh,
                                     const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_DOPS, state, "dops") {}
};

class SBPCallbackHandlerRelayPosEcef
    : public SBPCallbackHandlerRelay<msg_pos_ecef_t,
                                     piksi_multi_msgs::PosEcef> {
 public:
  inline SBPCallbackHandlerRelayPosEcef(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_POS_ECEF, state, "pos_ecef") {}
};

class SBPCallbackHandlerRelayPosEcefCov
    : public SBPCallbackHandlerRelay<msg_pos_ecef_cov_t,
                                     piksi_multi_msgs::PosEcefCov> {
 public:
  inline SBPCallbackHandlerRelayPosEcefCov(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_POS_ECEF_COV, state,
                                "pos_ecef_cov") {}
};

class SBPCallbackHandlerRelayPosLlh
    : public SBPCallbackHandlerRelay<msg_pos_llh_t, piksi_multi_msgs::PosLlh> {
 public:
  inline SBPCallbackHandlerRelayPosLlh(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_POS_LLH, state, "pos_llh") {}
};

class SBPCallbackHandlerRelayPosLlhCov
    : public SBPCallbackHandlerRelay<msg_pos_llh_cov_t,
                                     piksi_multi_msgs::PosLlhCov> {
 public:
  inline SBPCallbackHandlerRelayPosLlhCov(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_POS_LLH_COV, state, "pos_llh_cov") {
  }
};

class SBPCallbackHandlerRelayBaselineEcef
    : public SBPCallbackHandlerRelay<msg_baseline_ecef_t,
                                     piksi_multi_msgs::BaselineEcef> {
 public:
  inline SBPCallbackHandlerRelayBaselineEcef(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_BASELINE_ECEF, state,
                                "baseline_ecef") {}
};

class SBPCallbackHandlerRelayBaselineNed
    : public SBPCallbackHandlerRelay<msg_baseline_ned_t,
                                     piksi_multi_msgs::BaselineNed> {
 public:
  inline SBPCallbackHandlerRelayBaselineNed(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_BASELINE_NED, state,
                                "baseline_ned") {}
};

class SBPCallbackHandlerRelayVelEcef
    : public SBPCallbackHandlerRelay<msg_vel_ecef_t,
                                     piksi_multi_msgs::VelEcef> {
 public:
  inline SBPCallbackHandlerRelayVelEcef(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_ECEF, state, "vel_ecef") {}
};

class SBPCallbackHandlerRelayVelEcefCov
    : public SBPCallbackHandlerRelay<msg_vel_ecef_cov_t,
                                     piksi_multi_msgs::VelEcefCov> {
 public:
  inline SBPCallbackHandlerRelayVelEcefCov(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_ECEF_COV, state,
                                "vel_ecef_cov") {}
};

class SBPCallbackHandlerRelayVelNed
    : public SBPCallbackHandlerRelay<msg_vel_ned_t, piksi_multi_msgs::VelNed> {
 public:
  inline SBPCallbackHandlerRelayVelNed(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_NED, state, "vel_ned") {}
};

class SBPCallbackHandlerRelayVelNedCov
    : public SBPCallbackHandlerRelay<msg_vel_ned_cov_t,
                                     piksi_multi_msgs::VelNedCov> {
 public:
  inline SBPCallbackHandlerRelayVelNedCov(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_NED_COV, state, "vel_ned_cov") {
  }
};

class SBPCallbackHandlerRelayVelBody
    : public SBPCallbackHandlerRelay<msg_vel_body_t,
                                     piksi_multi_msgs::VelBody> {
 public:
  inline SBPCallbackHandlerRelayVelBody(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_VEL_BODY, state, "vel_body") {}
};

class SBPCallbackHandlerRelayAgeCorrections
    : public SBPCallbackHandlerRelay<msg_age_corrections_t,
                                     piksi_multi_msgs::AgeCorrections> {
 public:
  inline SBPCallbackHandlerRelayAgeCorrections(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_AGE_CORRECTIONS, state,
                                "age_corrections") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_NAVIGATION_H_
