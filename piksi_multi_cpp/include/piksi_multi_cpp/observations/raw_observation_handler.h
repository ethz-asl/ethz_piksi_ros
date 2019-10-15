#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_RAW_OBSERVATIONS_HANDLER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_RAW_OBSERVATIONS_HANDLER_H_
#include <piksi_multi_cpp/observations/observation_callback_interface.h>
#include <piksi_multi_cpp/observations/raw_observation_consumer.h>
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
  void observationCallback(msg_obs_t msg) final;
  void observationCallback(msg_heartbeat_t msg) final;

  static std::shared_ptr<CBtoRawObsConverter> createFor(
      const RawObservationInterface::Ptr& consumer) {
    auto instance = std::make_shared<CBtoRawObsConverter>();
    instance->raw_consumers_.push_back(consumer);
    return instance;
  }

 private:
  static int32_t sbp_write_redirect(uint8_t* buff, uint32_t n, void* context);
  uint16_t sbp_sender_id_{26};
  sbp_state_t sbp_state_;

  std::vector<RawObservationInterface::Ptr> raw_consumers_;
};
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_RAW_OBSERVATIONS_HANDLER_H_
