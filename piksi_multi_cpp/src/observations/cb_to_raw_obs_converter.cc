#include <piksi_multi_cpp/observations/cb_to_raw_obs_converter.h>
#include <ros/console.h>
namespace piksi_multi_cpp {
CBtoRawObsConverter::CBtoRawObsConverter() : sbp_state_{} {
  sbp_state_init(&sbp_state_);
  sbp_state_set_io_context(&sbp_state_, this);
}

void CBtoRawObsConverter::observationCallback(msg_base_pos_ecef_t msg) {
  startMessage();
  // Repack into full SBP Message
  sbp_send_message(&sbp_state_, SBP_MSG_BASE_POS_ECEF, sbp_sender_id_,
                   sizeof(msg), reinterpret_cast<uint8_t*>(&msg),
                   &CBtoRawObsConverter::sbp_write_redirect);
  // this triggers sbp_write_redirect
  finishMessage();
}

void CBtoRawObsConverter::observationCallback(msg_glo_biases_t msg) {
  // Repack into full SBP Message
  startMessage();
  sbp_send_message(&sbp_state_, SBP_MSG_GLO_BIASES, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &CBtoRawObsConverter::sbp_write_redirect);
  // this triggers sbp_write_redirect
  finishMessage();
}

void CBtoRawObsConverter::observationCallback(msg_obs_t_var msg) {
  // reconstruct raw buffer
  std::vector<uint8_t> observation_buffer;
  observation_buffer.resize(msg.length());
  msg.toBuffer(observation_buffer.data());

  // Repack into full SBP Message
  startMessage();
  sbp_send_message(&sbp_state_, SBP_MSG_OBS, sbp_sender_id_,
                   observation_buffer.size(),
                   reinterpret_cast<uint8_t*>(observation_buffer.data()),
                   &CBtoRawObsConverter::sbp_write_redirect);

  // this triggers sbp_write_redirect
  finishMessage();
}

void CBtoRawObsConverter::observationCallback(msg_heartbeat_t msg) {
  // Repack into full SBP Message
  startMessage();
  sbp_send_message(&sbp_state_, SBP_MSG_HEARTBEAT, sbp_sender_id_, sizeof(msg),
                   reinterpret_cast<uint8_t*>(&msg),
                   &CBtoRawObsConverter::sbp_write_redirect);
  // this triggers sbp_write_redirect
  finishMessage();
}

void CBtoRawObsConverter::startMessage() { buffer_msg_.clear(); }

void CBtoRawObsConverter::finishMessage() {
  // forward to all consumers.
  for (const auto& consumer : raw_consumers_) {
    consumer->insertObservation(buffer_msg_);
  }
}

int32_t CBtoRawObsConverter::sbp_write_redirect(uint8_t* buff, uint32_t n,
                                                void* context) {
  auto obj = static_cast<CBtoRawObsConverter*>(context);

  // write to buffer
  obj->buffer_msg_.insert(obj->buffer_msg_.end(), buff, buff + n);
  return n;
}
}  // namespace piksi_multi_cpp
