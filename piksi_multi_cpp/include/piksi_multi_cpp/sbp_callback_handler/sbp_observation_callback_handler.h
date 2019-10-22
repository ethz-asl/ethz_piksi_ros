#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBSERVATION_CALLBACK_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBSERVATION_CALLBACK_HANDLER_H_
#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <piksi_multi_cpp/observations/callback_observation_interface.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h>
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
class SBPObservationCallbackHandler {
 public:
  SBPObservationCallbackHandler(const ros::NodeHandle nh,
                                const std::shared_ptr<sbp_state_t>& state);

  void addObservationCallbackListener(
      const CallbackObservationInterface::Ptr& listener);

 private:
  /*
   * Convenience method to create a std::function for the callbackToListeners
   * method.
   * Implemented in header because of template.
   */
  template <class SBPMsgStruct>
  std::function<void(const SBPMsgStruct&, const uint8_t len)> getCallback() {
    return std::bind(
        &SBPObservationCallbackHandler::callbackToListeners<SBPMsgStruct>, this,
        s::_1, s::_2);
  }

  /*
   * Forwards messages to observation listeners.
   * Important: Only compiles if it is used only for sbp messages that can be
   * consumed by the listeners, i.e. that have a viable observationCallback
   * overload
   * Implemented in header because of template.
   */
  template <class SBPMsgStruct>
  void callbackToListeners(const SBPMsgStruct& msg, const uint8_t len) {
    // trigger callback on all listeners
    for (const auto& listener : listeners_) {
      listener->observationCallback(msg);
    }
  }

  std::shared_ptr<sbp_state_t> state_;
  std::vector<CallbackObservationInterface::Ptr> listeners_{};
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
