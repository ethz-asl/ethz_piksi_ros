#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_OBSERVATION_CALLBACK_INTERFACE_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_OBSERVATION_CALLBACK_INTERFACE_H_
#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <libsbp/system.h>
namespace piksi_multi_cpp {
class ObservationCallbackInterface {
  virtual void observationCallback(msg_base_pos_ecef_t msg) = 0;
  virtual void observationCallback(msg_glo_biases_t msg) = 0;
  virtual void observationCallback(msg_obs_t msg) = 0;
  virtual void observationCallback(msg_heartbeat_t msg) = 0;
};

}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_OBSERVATION_CALLBACK_INTERFACE_H_
