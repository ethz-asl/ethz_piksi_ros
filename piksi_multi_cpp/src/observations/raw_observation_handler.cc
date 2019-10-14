#include <piksi_multi_cpp/observations/raw_observation_handler.h>
namespace piksi_multi_cpp {
RawObservationHandler::RawObservationHandler() {
  sbp_state_init(&sbp_state_);
  sbp_state_set_io_context(&sbp_state_, this);
}

void RawObservationHandler::observationCallback(msg_base_pos_ecef_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_BASE_POS_ECEF, sbp_sender_id_,
                   sizeof(msg), reinterpret_cast<uint8_t*>(&msg),
                   &RawObservationHandler::sbp_write_redirect);
  // this triggers sbp_write_redirect, which triggers write
}

void RawObservationHandler::observationCallback(msg_glo_biases_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_GLO_BIASES, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &RawObservationHandler::sbp_write_redirect);
  // this triggers sbp_write_redirect, which triggers write
}

void RawObservationHandler::observationCallback(msg_obs_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_OBS, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &RawObservationHandler::sbp_write_redirect);
  // this triggers sbp_write_redirect, which triggers write
}

void RawObservationHandler::observationCallback(msg_heartbeat_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_HEARTBEAT, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &RawObservationHandler::sbp_write_redirect);
  // this triggers sbp_write_redirect, which triggers write
}

int32_t RawObservationHandler::sbp_write_redirect(uint8_t* buff, uint32_t n,
                                                  void* context) {
  RawObservationHandler* obj = static_cast<RawObservationHandler*>(context);
  std::vector<uint8_t> data;
  data.insert(data.end(), buff, buff + n);
  obj->write(data);
  return n;
}
}  // namespace piksi_multi_cpp
