#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBSERVATION_CALLBACK_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBSERVATION_CALLBACK_HANDLER_H_
#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <piksi_multi_cpp/observations/callback_msg_interface.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_msg_type_callback_handler.h>
#include <ros/ros.h>
#include <memory>
#include <utility>

namespace piksi_multi_cpp {
namespace s = std::placeholders;

/*
 * Subscribes to all 4 observation type callbacks simultaneously using
 * LambdaCallbackHandlers and redirects these messages to all registered
 * listeners.
 *
 */
class SBPObservationCallbackHandler
    : public SBPMsgTypeCallbackHandler {
 public:
  SBPObservationCallbackHandler(const ros::NodeHandle nh,
                                const std::shared_ptr<sbp_state_t>& state)
      : SBPMsgTypeCallbackHandler(nh, state),
        base_pos_handler_{getCallback<msg_base_pos_ecef_t>(),
                          SBP_MSG_BASE_POS_ECEF, state},
        glo_bias_handler_{getCallback<msg_glo_biases_t>(), SBP_MSG_GLO_BIASES,
                          state},
        obs_handler_{getCallback<msg_obs_t_var>(), SBP_MSG_OBS, state},
        heartbeat_handler_{getCallback<msg_heartbeat_t>(), SBP_MSG_HEARTBEAT,
                           state} {};

 private:
  /* Set up Lambda redirect callbacks for the four message types used for
   * corrections, such that this class gets any of the four callbacks and can
   * forward them to the observation listeners */
  SBPLambdaCallbackHandler<msg_base_pos_ecef_t> base_pos_handler_;
  SBPLambdaCallbackHandler<msg_glo_biases_t> glo_bias_handler_;
  SBPLambdaCallbackHandler<msg_obs_t_var> obs_handler_;
  SBPLambdaCallbackHandler<msg_heartbeat_t> heartbeat_handler_;
};
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBSERVATION_CALLBACK_HANDLER_H_
