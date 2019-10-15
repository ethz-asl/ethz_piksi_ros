#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_OBSERVATION_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_OBSERVATION_H_

#include <libsbp/observation.h>
#include <piksi_multi_msgs/Obs.h>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_callback_handler_relay.h"

namespace piksi_multi_cpp {

class SBPCallbackHandlerRelayObs
    : public SBPCallbackHandlerRelay<msg_obs_t, piksi_multi_msgs::Obs> {
 public:
  inline SBPCallbackHandlerRelayObs(const ros::NodeHandle& nh,
                                    const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_OBS, state, "obs") {}
};

class SBPCallbackHandlerRelayBasePosLlh
    : public SBPCallbackHandlerRelay<msg_base_pos_llh_t,
                                     piksi_multi_msgs::PointWgs84> {
 public:
  inline SBPCallbackHandlerRelayBasePosLlh(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_BASE_POS_LLH, state,
                                "base_pos_llh") {}
};

class SBPCallbackHandlerRelayBasePosEcef
    : public SBPCallbackHandlerRelay<msg_base_pos_ecef_t,
                                     geometry_msgs::Point> {
 public:
  inline SBPCallbackHandlerRelayBasePosEcef(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandlerRelay(nh, SBP_MSG_BASE_POS_ECEF, state,
                                "base_pos_ecef") {}
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_OBSERVATION_H_
