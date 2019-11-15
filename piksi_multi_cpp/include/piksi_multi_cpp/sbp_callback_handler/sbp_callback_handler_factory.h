#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_FACTORY_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_FACTORY_H_

#include <libsbp/sbp.h>
#include <memory>
#include <vector>
#include "piksi_multi_cpp/sbp_callback_handler/geotf_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/ros_time_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"

namespace piksi_multi_cpp {
/* A factory to help initiating sbp callback handlers at runtime.
A possible usecase is where the receiver wants to relay all messages currently
outputted by the Piksi.
See also http://www.blackwasp.co.uk/FactoryMethod.aspx for more details on
factories. */
class SBPCallbackHandlerFactory {
 public:
  // Factory method to create all implemented SBP message relays.
  static std::vector<SBPCallbackHandler::Ptr> createAllSBPMessageRelays(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state);

  static std::vector<SBPCallbackHandler::Ptr> createAllRosMessageRelays(
      const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state,
      const RosTimeHandler::Ptr& ros_time_handler,
      const GeoTfHandler::Ptr& geotf_handler);
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_CALLBACK_HANDLER_FACTORY_H_
