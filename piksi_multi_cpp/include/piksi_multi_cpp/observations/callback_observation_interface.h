#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_CALLBACK_OBSERVATION_INTERFACE_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_CALLBACK_OBSERVATION_INTERFACE_H_
#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_additional_msgs.h>
#include <memory>

namespace piksi_multi_cpp {
/*
 * Interface for a class that can handle observation callbacks
 * from SBP.
 */
class CallbackObservationInterface {
 public:
  typedef std::shared_ptr<CallbackObservationInterface> Ptr;
  virtual void observationCallback(msg_base_pos_ecef_t msg) = 0;
  virtual void observationCallback(msg_glo_biases_t msg) = 0;
  virtual void observationCallback(msg_obs_t_var msg) = 0;
  virtual void observationCallback(msg_heartbeat_t msg) = 0;
};

}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_CALLBACK_OBSERVATION_INTERFACE_H_
