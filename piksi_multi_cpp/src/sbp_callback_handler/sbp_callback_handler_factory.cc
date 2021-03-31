#include <libsbp_ros_msgs/sbp_relays.h>
#include <memory>
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h"

#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_base_pos_relay.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_enu_relays.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_ext_event_relay.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_imu_relay.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_mag_relay.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_receiver_state.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_relays.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/ros_time_ref_relay.h"

namespace piksi_multi_cpp {

// Factory method to create all implemented SBP message relays.
std::vector<SBPCallbackHandler::Ptr>
SBPCallbackHandlerFactory::createAllSBPMessageRelays(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  return createAllSbpRelays(nh, state);
}

std::vector<SBPCallbackHandler::Ptr>
SBPCallbackHandlerFactory::createAllRosMessageRelays(
    const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state,
    const RosTimeHandler::Ptr& ros_time_handler,
    const GeoTfHandler::Ptr& geotf_handler) {
  // Create all relays.
  std::vector<SBPCallbackHandler::Ptr> relays;

  relays.push_back(SBPCallbackHandler::Ptr(
      new RosExtEventRelay(nh, state, ros_time_handler)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosPosEcefRelay(nh, state, ros_time_handler)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosPosEcefCovRelay(nh, state, ros_time_handler)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosBaselineNedRelay(nh, state, ros_time_handler)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosVelEcefRelay(nh, state, ros_time_handler)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosVelEcefCovRelay(nh, state, ros_time_handler)));
  relays.push_back(
      SBPCallbackHandler::Ptr(new RosVelNedRelay(nh, state, ros_time_handler)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosVelNedCovRelay(nh, state, ros_time_handler)));
  relays.push_back(
      SBPCallbackHandler::Ptr(new RosImuRelay(nh, state, ros_time_handler)));
  relays.push_back(
      SBPCallbackHandler::Ptr(new RosMagRelay(nh, state, ros_time_handler)));
  relays.push_back(SBPCallbackHandler::Ptr(new RosBasePosEcefRelay(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosBasePosLlhRelay(nh, state, geotf_handler)));
  relays.push_back(
      SBPCallbackHandler::Ptr(new RosTimeReferenceRelay(nh, state)));

  auto ros_receiver_state =
      std::make_shared<RosReceiverState>(nh, state, ros_time_handler);
  relays.push_back(ros_receiver_state);
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosPosLlhCovRelay(nh, state, ros_time_handler, ros_receiver_state)));

  relays.push_back(SBPCallbackHandler::Ptr(
      new RosPosEnuRelay(nh, state, ros_time_handler, geotf_handler)));
  relays.push_back(SBPCallbackHandler::Ptr(
      new RosTransformEnuRelay(nh, state, ros_time_handler, geotf_handler)));
  relays.push_back(
      SBPCallbackHandler::Ptr(new RosPositionWithCovarianceEnuRelay(
          nh, state, ros_time_handler, geotf_handler)));
  relays.push_back(SBPCallbackHandler::Ptr(new RosPoseWithCovarianceEnuRelay(
      nh, state, ros_time_handler, geotf_handler)));

  return relays;
}

}  // namespace piksi_multi_cpp
