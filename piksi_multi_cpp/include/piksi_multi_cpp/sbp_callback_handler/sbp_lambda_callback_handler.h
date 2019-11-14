#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_LAMBDA_CALLBACK_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_LAMBDA_CALLBACK_HANDLER_H_
#include <libsbp/sbp.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_additional_msgs.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h>
#include <ros/console.h>
#include <condition_variable>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

namespace piksi_multi_cpp {

/*
 * Callback Handler that can be redirected to a lambda expression
 * or std::bind expression.
 *
 * Implementation of methods have to be in the header (we don't want to do
 * explicit template instantiation)
 */
template <class SBPMsgStruct>
class SBPLambdaCallbackHandler : SBPCallbackHandler {
 public:
  typedef std::function<void(const SBPMsgStruct&, const uint8_t)> CallbackFn;

  SBPLambdaCallbackHandler(const CallbackFn func, const uint16_t msg_id,
                           const std::shared_ptr<sbp_state_t>& state)
      : SBPCallbackHandler(msg_id, state), callback_redirect_(func) {}

 public:
  bool waitForCallback(int timeout = 1000) {
    std::unique_lock<std::mutex> lock(callback_mutex_);
    auto now = std::chrono::system_clock::now();
    return cv_.wait_until(lock, now + std::chrono::milliseconds(timeout),
                          [this]() { return callback_received_.load(); });
  }

 private:
  // Disable copy / assignement constructors.
  // TODO(rikba): Can we make this compatible with std::optional?
  SBPLambdaCallbackHandler(const SBPLambdaCallbackHandler&) = delete;
  SBPLambdaCallbackHandler& operator=(const SBPLambdaCallbackHandler&) = delete;

  void callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) override {
    /* Parse message to type*/
    auto sbp_msg = (SBPMsgStruct*)msg;
    if (!sbp_msg) {
      ROS_WARN("Cannot cast SBP message.");
      return;
    }
    callback_redirect_(*sbp_msg, len);
    callback_received_ = true;
    cv_.notify_all();
  }

  CallbackFn callback_redirect_;
  std::condition_variable cv_;
  std::atomic_bool callback_received_{false};
  std::mutex callback_mutex_;
};

template <>
inline void SBPLambdaCallbackHandler<msg_obs_t_var>::callback(
    uint16_t sender_id, uint8_t len, uint8_t* msg) {
  msg_obs_t_var obs_msg;
  obs_msg.fromBuffer(msg, len);
  callback_redirect_(obs_msg, len);
}
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_LAMBDA_CALLBACK_HANDLER_H_
