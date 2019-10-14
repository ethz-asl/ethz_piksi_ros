#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_RAW_OBSERVATIONS_HANDLER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_RAW_OBSERVATIONS_HANDLER_H_
#include <piksi_multi_cpp/observations/observation_callback_interface.h>
#include <vector>

namespace piksi_multi_cpp {
class RawObservationHandler : public ObservationCallbackInterface {
 public:
  RawObservationHandler();
  void observationCallback(msg_base_pos_ecef_t msg) final;
  void observationCallback(msg_glo_biases_t msg) final;
  void observationCallback(msg_obs_t msg) final;
  void observationCallback(msg_heartbeat_t msg) final;

 protected:
  virtual void write(std::vector<uint8_t> data) = 0;

 private:
  static int32_t sbp_write_redirect(uint8_t* buff, uint32_t n, void* context);
  uint16_t sbp_sender_id_{26};
  sbp_state_t sbp_state_;
};
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_RAW_OBSERVATIONS_HANDLER_H_
