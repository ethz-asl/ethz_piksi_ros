#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBSERVATION_CALLBACK_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBSERVATION_CALLBACK_HANDLER_H_
#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <piksi_multi_cpp/observations/observation_callback_interface.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h>
#include <ros/ros.h>
#include <memory>
#include <utility>

namespace piksi_multi_cpp {
namespace s = std::placeholders;

class SBPObservationCallbackHandler {
 public:
  SBPObservationCallbackHandler(const ros::NodeHandle nh,
                                const std::shared_ptr<sbp_state_t>& state)
      : state_(state) {}

  void addObservationCallbackListener(
      const ObservationCallbackInterface::Ptr& listener) {
    if (listener.get()) {
      listeners_.push_back(listener);
    }
  }

 private:
  virtual void callbackBasePosEcef(const msg_base_pos_ecef_t& msg) {
    for (const auto& listener : listeners_) {
      listener->observationCallback(msg);
    }
  };

  virtual void callbackGloBiases(const msg_glo_biases_t& msg) {
    for (const auto& listener : listeners_) {
      listener->observationCallback(msg);
    }
  };

  virtual void callbackObs(const msg_obs_t& msg) {
    for (const auto& listener : listeners_) {
      listener->observationCallback(msg);
    }
  };
  virtual void callbackHeartbeat(const msg_heartbeat_t& msg) {
    for (const auto& listener : listeners_) {
      listener->observationCallback(msg);
    }
  };

  /* Callback redirects that support multiple callbacks */
  SBPLambdaCallbackHandler<msg_base_pos_ecef_t> base_pos_handler_{
      std::bind(&SBPObservationCallbackHandler::callbackBasePosEcef, this,
                s::_1),
      SBP_MSG_BASE_POS_ECEF, state_};

  SBPLambdaCallbackHandler<msg_glo_biases_t> glo_bias_handler_{
      std::bind(&SBPObservationCallbackHandler::callbackGloBiases, this, s::_1),
      SBP_MSG_GLO_BIASES, state_};

  SBPLambdaCallbackHandler<msg_obs_t> obs_handler_{
      std::bind(&SBPObservationCallbackHandler::callbackObs, this, s::_1),
      SBP_MSG_OBS, state_};

  SBPLambdaCallbackHandler<msg_heartbeat_t> heartbeat_handler_{
      std::bind(&SBPObservationCallbackHandler::callbackHeartbeat, this, s::_1),
      SBP_MSG_HEARTBEAT, state_};

  std::shared_ptr<sbp_state_t> state_;
  std::vector<ObservationCallbackInterface::Ptr> listeners_;
};
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBSERVATION_CALLBACK_HANDLER_H_
