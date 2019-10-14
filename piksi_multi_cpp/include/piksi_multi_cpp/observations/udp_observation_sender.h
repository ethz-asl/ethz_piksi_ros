#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATIONS_SENDER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATIONS_SENDER_H_
#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <vector>

namespace piksi_multi_cpp {
class UDPObservationsSender {
 public:
  UDPObservationsSender();
  void distributeMessage(msg_base_pos_ecef_t msg);
  void distributeMessage(msg_glo_biases_t msg);
  void distributeMessage(msg_obs_t msg);
  void distributeMessage(msg_heartbeat_t msg);

 private:
  void sendUDPPacket(std::vector<uint8_t> data);

  static int32_t sbp_write_redirect(uint8_t* buff, uint32_t n, void* context);
  uint16_t sbp_sender_id_{26};
  sbp_state_t sbp_state_;
};
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATIONS_SENDER_H_
