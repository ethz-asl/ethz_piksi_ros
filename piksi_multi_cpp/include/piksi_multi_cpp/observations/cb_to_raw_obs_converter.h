#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_CB_TO_RAW_OBS_CONVERTER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_CB_TO_RAW_OBS_CONVERTER_H_
#include <piksi_multi_cpp/observations/callback_observation_interface.h>
#include <piksi_multi_cpp/observations/raw_observation_interface.h>
#include <vector>

namespace piksi_multi_cpp {

/*
 * Class that can recreate raw sbp messages from callbacks.
 * This sounds redundant, but is actually needed. Why?
 * The Observation Messages that are passed through UDP or a filewriter have to
 * be valid sbp messages. But the structs we're getting from the callbacks are
 * already unpacked, so we repack them again.
 *
 * It also creates its own sbp_state, as this is very different from the other
 * usecases.
 *
 * It of course can forward the repacked raw observations to any object that has
 * the RawObservationInterface
 *
 */
class CBtoRawObsConverter : public CallbackObservationInterface {
 public:
  typename std::shared_ptr<CBtoRawObsConverter> Ptr;
  CBtoRawObsConverter();
  void observationCallback(msg_base_pos_ecef_t msg) final;
  void observationCallback(msg_glo_biases_t msg) final;
  void observationCallback(msg_obs_t_var msg) final;
  void observationCallback(msg_heartbeat_t msg) final;

  static std::shared_ptr<CBtoRawObsConverter> createFor(
      const RawObservationInterface::Ptr& consumer, const uint16_t sbp_sender_id) {
    auto instance = std::make_shared<CBtoRawObsConverter>();
    instance->raw_consumers_.push_back(consumer);

    // In later piksi versions, this needs to be set.
    instance->sbp_sender_id_ = sbp_sender_id;
    return instance;
  }

 private:
  static int32_t sbp_write_redirect(uint8_t* buff, uint32_t n, void* context);

  /*
   * One problem with the SBP library is that when sending packets,
   * "sbp_write_redirect" gets called multiple times per message - once for the
   * header, the checksum, the payload etc. So here we use start / finishMessage
   * to clear the buffer before a new message is written and to send the buffer
   * to consumers once the message is done.
   * Inside the callbacks, the call order is as follows:
   * 1. call startMessage() to empty buffers
   * 2. call sbp_send_message to trigger encapsulation of SBP messages and cause
   *    multiple calls to sbp_write_redirect
   * 3. call finishMessage() to finally send the complete, buffered SBP message
   *    to all consumers.
   *
   * NOTE: NOT THREADSAFE, only use in single threaded environment.
   */
  void startMessage();
  void finishMessage();

  uint16_t sbp_sender_id_{0x42};
  sbp_state_t sbp_state_;

  RawObservation buffer_msg_;
  std::vector<RawObservationInterface::Ptr> raw_consumers_{};
};
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_CB_TO_RAW_OBS_CONVERTER_H_
