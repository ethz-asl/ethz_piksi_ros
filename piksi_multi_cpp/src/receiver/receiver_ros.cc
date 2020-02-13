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

  if (1) {
    auto logger = std::make_shared<FileObservationLogger>();
    /*ROS_WARN_STREAM(logger->open("/tmp/tempfile.sbp"));
    obs_cbs_->addObservationCallbackListener(
        CBtoRawObsConverter::createFor(logger));*/
  }
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
