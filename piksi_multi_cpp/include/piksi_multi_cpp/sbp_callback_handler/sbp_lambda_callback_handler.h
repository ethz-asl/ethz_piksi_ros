
#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_LAMBDA_CALLBACK_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_LAMBDA_CALLBACK_HANDLER_H_
#include <libsbp/sbp.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h>

#include <memory>

namespace piksi_multi_cpp {

template <class SBPMsgStruct>
class SBPLambdaCallbackHandler : SBPCallbackHandler {
 public:
  typedef std::function<void(const SBPMsgStruct&)> CallbackFn;

  SBPLambdaCallbackHandler(const CallbackFn func, const uint16_t msg_id,
                           const std::shared_ptr<sbp_state_t>& state)
      : callback_redirect_(func),
        SBPCallbackHandler(ros::NodeHandle(), msg_id, state)
  {}

 private:
  // Disable copy / assignement constructors.
  SBPLambdaCallbackHandler(const SBPLambdaCallbackHandler&) = delete;
  SBPLambdaCallbackHandler& operator=(const SBPLambdaCallbackHandler&) = delete;

  void callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) {
    /* Parse message to type*/
    auto sbp_msg = (SBPMsgStruct*)msg;
    if (!sbp_msg) {
      ROS_WARN("Cannot cast SBP message.");
      return;
    }
    callback_redirect_(*sbp_msg);
  }

  CallbackFn callback_redirect_;
};
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_LAMBDA_CALLBACK_HANDLER_H_
