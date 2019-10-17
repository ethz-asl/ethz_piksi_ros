#include <piksi_multi_cpp/sbp_callback_handler/sbp_observation_callback_handler.h>

namespace piksi_multi_cpp {

SBPObservationCallbackHandler::SBPObservationCallbackHandler(
    const ros::NodeHandle nh, const std::shared_ptr<sbp_state_t>& state)
    : state_(state),
      base_pos_handler_{getCallback<msg_base_pos_ecef_t>(),
                        SBP_MSG_BASE_POS_ECEF, state},
      glo_bias_handler_{getCallback<msg_glo_biases_t>(), SBP_MSG_GLO_BIASES,
                        state},
      obs_handler_{getCallback<msg_obs_t_var>(), SBP_MSG_OBS, state},
      heartbeat_handler_{getCallback<msg_heartbeat_t>(), SBP_MSG_HEARTBEAT,
                         state} {}

void SBPObservationCallbackHandler::addObservationCallbackListener(
    const CallbackObservationInterface::Ptr& listener) {
  if (listener.get()) {
    listeners_.push_back(listener);
  }
}

}  // namespace piksi_multi_cpp
