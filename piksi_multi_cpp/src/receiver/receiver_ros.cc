#include "piksi_multi_cpp/receiver/receiver_ros.h"

#include <piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h>

// SBP message definitions.
#include <piksi_multi_cpp/observations/file_observation_logger.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include "piksi_multi_cpp/sbp_callback_handler/position_sampler.h"
#include "piksi_multi_cpp/sbp_callback_handler/ros_time_handler.h"

namespace piksi_multi_cpp {

ReceiverRos::ReceiverRos(const ros::NodeHandle& nh, const Device::Ptr& device)
    : SettingsIo(device), nh_(nh) {
  // Register all relay callbacks.
  // Handle (GPS) time stamping.
  auto ros_time_handler = std::make_shared<RosTimeHandler>(state_);
  geotf_handler_ = std::make_shared<GeoTfHandler>(nh, state_);
  sbp_relays_ =
      SBPCallbackHandlerFactory::createAllSBPMessageRelays(nh, state_);
  ros_relays_ = SBPCallbackHandlerFactory::createAllRosMessageRelays(
      nh, state_, ros_time_handler, geotf_handler_);
  position_sampler_ = std::make_shared<PositionSampler>(
      nh, state_, ros_time_handler, geotf_handler_);

  // Create observation callbacks
  obs_cbs_ = std::make_unique<SBPObservationCallbackHandler>(nh, state_);
}

bool ReceiverRos::init() {
  if (!Receiver::init()) {
    return false;
  }

  // Get and store ID of device
  while (!readSetting("system_info", "sbp_sender_id")) {
  }
  sbp_sender_id_ = static_cast<uint16_t>(std::stoul(getValue(), nullptr, 16));

  return true;
}

void ReceiverRos::startFileLogger(const std::string& log_file_dir) {
  // Create ephemeris callbacks
  eph_cbs_ = std::make_unique<SBPEphemerisCallbackHandler>(nh_, state_);

  auto logger = std::make_shared<FileObservationLogger>();
  ROS_WARN_STREAM(logger->open(log_file_dir));

  // Add logger as listener to callbacks
  obs_cbs_->addMsgCallbackListener(CBtoRawObsConverter::createFor(
      logger, sbp_sender_id_));  
  eph_cbs_->addMsgCallbackListener(
      CBtoRawObsConverter::createFor(logger, sbp_sender_id_));
}

std::vector<std::string> ReceiverRos::getVectorParam(
    const std::string& name, const std::string& default_value) {
  auto string_value = nh_.param<std::string>(name, default_value);
  if (string_value.length() == 0) return {};

  std::vector<std::string> vector_value;
  boost::algorithm::split(vector_value, string_value, boost::is_any_of(";"));
  return vector_value;
}

}  // namespace piksi_multi_cpp
