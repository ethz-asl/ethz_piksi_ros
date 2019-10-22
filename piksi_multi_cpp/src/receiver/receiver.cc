#include "piksi_multi_cpp/receiver/receiver.h"

#include <libsbp/sbp.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h>

// SBP message definitions.
#include <libsbp/system.h>
#include <piksi_multi_cpp/observations/file_observation_logger.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

namespace piksi_multi_cpp {

Receiver::Receiver(const ros::NodeHandle& nh, const Device::Ptr& device)
    : nh_(nh), device_(device), thread_exit_requested_(false) {
  // Initialize SBP state.
  state_ = std::make_shared<sbp_state_t>();
  sbp_state_init(state_.get());
  // Pass device pointer to process function.
  sbp_state_set_io_context(state_.get(), device_.get());

  // Register all relay callbacks.
  sbp_relays_ =
      SBPCallbackHandlerFactory::createAllSBPMessageRelays(nh, state_);
  ros_relays_ =
      SBPCallbackHandlerFactory::createAllRosMessageRelays(nh, state_);

  // Create observation callbacks
  obs_cbs_ = std::make_unique<SBPObservationCallbackHandler>(nh, state_);

  if (1) {
    auto logger = std::make_shared<FileObservationLogger>();
    /*ROS_WARN_STREAM(logger->open("/tmp/tempfile.sbp"));
    obs_cbs_->addObservationCallbackListener(
        CBtoRawObsConverter::createFor(logger));*/
  }
}

std::vector<std::string> Receiver::getVectorParam(
    const std::string& name, const std::string& default_value) {
  auto string_value = nh_.param<std::string>(name, default_value);
  if (string_value.length() == 0) return {};

  std::vector<std::string> vector_value;
  boost::algorithm::split(vector_value, string_value, boost::is_any_of(";"));
  return vector_value;
}

Receiver::~Receiver() {
  // Close thread.
  thread_exit_requested_.store(true);
  if (process_thread_.joinable()) process_thread_.join();

  if (device_) device_->close();
}

bool Receiver::init() {
  if (!device_.get()) {
    ROS_ERROR("Device not set.");
    return false;
  }

  // Open attached device.
  if (!device_->open()) {
    ROS_ERROR_STREAM("Cannot open device with id " << device_->getID());
    return false;
  }

  process_thread_ = std::thread(&Receiver::process, this);

  return true;
}

bool Receiver::isRunning() const { return process_thread_.joinable(); }

void Receiver::process() {
  // Setting thread_exit_requested_ will terminate the thread.
  while (!thread_exit_requested_.load()) {
    if (!device_.get()) return;
    // Pass device read function to sbp_process.
    int result =
        sbp_process(state_.get(), &piksi_multi_cpp::Device::read_redirect);
    if (result < 0) {
      ROS_WARN_STREAM("Error sbp_process: " << result);
    }
  }
}

}  // namespace piksi_multi_cpp
