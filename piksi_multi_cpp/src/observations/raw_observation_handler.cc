#include <piksi_multi_cpp/observations/raw_observation_handler.h>
namespace piksi_multi_cpp {
CBtoRawObsConverter::CBtoRawObsConverter() {
  sbp_state_init(&sbp_state_);
  sbp_state_set_io_context(&sbp_state_, this);
}

void CBtoRawObsConverter::observationCallback(
    msg_base_pos_ecef_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_BASE_POS_ECEF, sbp_sender_id_,
                   sizeof(msg), reinterpret_cast<uint8_t*>(&msg),
                   &CBtoRawObsConverter::sbp_write_redirect);
  // this triggers sbp_write_redirect
}

void CBtoRawObsConverter::observationCallback(msg_glo_biases_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_GLO_BIASES, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &CBtoRawObsConverter::sbp_write_redirect);
  // this triggers sbp_write_redirect
}

void CBtoRawObsConverter::observationCallback(msg_obs_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_OBS, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &CBtoRawObsConverter::sbp_write_redirect);
  // this triggers sbp_write_redirect
}

void CBtoRawObsConverter::observationCallback(msg_heartbeat_t msg) {
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_HEARTBEAT, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &CBtoRawObsConverter::sbp_write_redirect);
  // this triggers sbp_write_redirect
}

int32_t CBtoRawObsConverter::sbp_write_redirect(uint8_t* buff,
                                                         uint32_t n,
                                                         void* context) {
  CBtoRawObsConverter* obj =
      static_cast<CBtoRawObsConverter*>(context);
  RawObservation data;
  data.insert(data.end(), buff, buff + n);

  // forward to all consumers.
  for (const auto& consumer : obj->raw_consumers_) {
    consumer->insertObservation(data);
  }

  return n;
}
}  // namespace piksi_multi_cpp
