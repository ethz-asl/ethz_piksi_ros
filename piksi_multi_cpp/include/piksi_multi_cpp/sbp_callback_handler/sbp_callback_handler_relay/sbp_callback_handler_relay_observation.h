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

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_RELAY_OBSERVATION_H_
