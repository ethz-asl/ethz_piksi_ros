#include <piksi_multi_cpp/sbp_callback_handler/sbp_observation_callback_handler.h>

namespace piksi_multi_cpp {

SBPObservationCallbackHandler::SBPObservationCallbackHandler(
    const ros::NodeHandle nh, const std::shared_ptr<sbp_state_t>& state)
    : state_(state) {}

void SBPObservationCallbackHandler::addObservationCallbackListener(
    const ObservationCallbackInterface::Ptr& listener) {
  if (listener.get()) {
    listeners_.push_back(listener);
  }
}

}  // namespace piksi_multi_cpp
