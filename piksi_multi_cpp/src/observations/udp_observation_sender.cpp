#include <piksi_multi_cpp/observations/udp_observation_sender.h>
namespace piksi_multi_cpp {
UDPObservationsSender::UDPObservationsSender() {
  sbp_state_init(&sbp_state_);
  sbp_state_set_io_context(&sbp_state_, this);
}

void UDPObservationsSender::distributeMessage(msg_base_pos_ecef_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_BASE_POS_ECEF, sbp_sender_id_,
                   sizeof(msg), reinterpret_cast<uint8_t*>(&msg),
                   &UDPObservationsSender::sbp_write_redirect);
  // this triggers sbp_write_redirect, which triggers sendUDPPacket
}

void UDPObservationsSender::distributeMessage(msg_glo_biases_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_GLO_BIASES, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &UDPObservationsSender::sbp_write_redirect);
  // this triggers sbp_write_redirect, which triggers sendUDPPacket
}

void UDPObservationsSender::distributeMessage(msg_obs_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_OBS, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &UDPObservationsSender::sbp_write_redirect);
  // this triggers sbp_write_redirect, which triggers sendUDPPacket
}

void UDPObservationsSender::distributeMessage(msg_heartbeat_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_HEARTBEAT, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &UDPObservationsSender::sbp_write_redirect);
  // this triggers sbp_write_redirect, which triggers sendUDPPacket
}

void UDPObservationsSender::sendUDPPacket(std::vector<uint8_t> data) {
  // send to socket.
}

int32_t UDPObservationsSender::sbp_write_redirect(uint8_t* buff, uint32_t n,
                                                  void* context) {
  UDPObservationsSender* obj = static_cast<UDPObservationsSender*>(context);
  std::vector<uint8_t> data;
  data.insert(data.end(), buff, buff + n);
  obj->sendUDPPacket(data);
  return n;
}
}  // namespace piksi_multi_cpp
