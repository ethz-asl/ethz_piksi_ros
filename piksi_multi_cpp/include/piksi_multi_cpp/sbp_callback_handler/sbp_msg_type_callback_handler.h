#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBS_TYPE_CALLBACK_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_OBS_TYPE_CALLBACK_HANDLER_H_
#include <libsbp/sbp.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h>
#include <ros/ros.h>

#include <piksi_multi_cpp/observations/callback_msg_interface.h>
#include <functional>

namespace piksi_multi_cpp {
namespace s = std::placeholders;

class SBPMsgTypeCallbackHandler {

 public:
  SBPMsgTypeCallbackHandler(const ros::NodeHandle nh,
                            const std::shared_ptr<sbp_state_t>& state)
      : state_(state){};

  /*
   * Add listeners to message callbacks specified in interface.
   */
  void addMsgCallbackListener(const CallbackMsgInterface::Ptr& listener) {
    if (listener.get()) {
      listeners_.push_back(listener);
    }
  }

 protected:
  /*
   * Convenience method to create a std::function for the callbackToListeners
   * method.
   * Implemented in header because of template.
   */
  template <class SBPMsgStruct>
  std::function<void(const SBPMsgStruct&, const uint8_t len)> getCallback() {
    return std::bind(
        &SBPMsgTypeCallbackHandler::callbackToListeners<SBPMsgStruct>, this,
        s::_1, s::_2);
  }

  /*
   * Forwards messages to observation listeners.
   * Important: Only compiles if it is used only for sbp messages that can be
   * consumed by the listeners, i.e. that have a viable messageCallback
   * overload
   * Implemented in header because of template.
   */
  template <class SBPMsgStruct>
  void callbackToListeners(const SBPMsgStruct& msg, const uint8_t len) {
    // trigger callback on all listeners
    for (const auto& listener : listeners_) {
      listener->messageCallback(msg);
    }
  }

  std::shared_ptr<sbp_state_t> state_;
  std::vector<CallbackMsgInterface::Ptr> listeners_{};
};
}  // namespace piksi_multi_cpp

#endif