#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_FACTORY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_FACTORY_H_

#include "piksi_multi_cpp/receiver/receiver.h"

namespace piksi_multi_cpp {
/* A factory to help initiating sbp callback handlers at runtime.
A possible usecase is where the receiver wants to relay all messages currently
outputted by the Piksi.
See also http://www.blackwasp.co.uk/FactoryMethod.aspx for more details on
factories. */
class SBPCallbackHandlerFactory {
 public:
  // Factory method to create a callback that relays a specified sbp_msg_type.
  static SBPCallbackHandler::SBPCallbackHandlerPtr
  createSBPRelayCallbackBySBPMsgType(const ros::NodeHandle& nh,
                                     const uint16_t sbp_msg_type,
                                     const std::shared_ptr<sbp_state_t>& state);
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_FACTORY_H_
