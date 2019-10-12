#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_H_

#include <libsbp/sbp.h>
#include <ros/ros.h>
#include <memory>

namespace piksi_multi_cpp {

const uint32_t kQueueSize = 100;
const bool kLatchTopic = true;

// This class handles the callback creation for the different piksi message
// types.
class SBPCallbackHandler {
 public:
  typedef std::shared_ptr<SBPCallbackHandler> Ptr;
  // Register callback.
  SBPCallbackHandler(const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
                     const std::shared_ptr<sbp_state_t>& state);

 protected:
  // Implement the specific callback here.
  virtual void callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) = 0;
  // A nodehandle with the correct ROS namespace.
  ros::NodeHandle nh_;

 private:
  // This function will be passed to sbp_register_callback.
  // libsbp is a C interface and does not allow binding a non-static member
  // function using std::bind. Thus we bind this static member function that
  // points on the context's callbackSBP function.
  static void callback_redirect(uint16_t sender_id, uint8_t len, uint8_t msg[],
                                void* context);

  sbp_msg_callbacks_node_t sbp_msg_callback_node_;
  std::shared_ptr<sbp_state_t> state_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_H_
