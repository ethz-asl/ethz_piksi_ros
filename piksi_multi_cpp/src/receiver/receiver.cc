#include "piksi_multi_cpp/receiver/receiver.h"

#include <libsbp/sbp.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_factory.h>
#include <piksi_rtk_msgs/Heartbeat.h>

// SBP message definitions.
#include <libsbp/system.h>

namespace piksi_multi_cpp {

Receiver::Receiver(const ros::NodeHandle& nh, const Device::Ptr& device)
    : nh_(nh), device_(device), thread_exit_requested_(false) {
  // Initialize SBP state.
  state_ = std::make_shared<sbp_state_t>();
  sbp_state_init(state_.get());
  // Register all relay callbacks.
  relay_cbs_ = SBPCallbackHandlerFactory::createAllSBPMessageRelays(nh, state_);
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
    ROS_ERROR("Cannot open device.");
    return false;
  }

  process_thread_ = std::thread(&Receiver::process, this);

  return true;
}

void Receiver::process() {
  // Setting thread_exit_requested_ will terminate the thread.
  while (!thread_exit_requested_.load()) {
    if (!device_.get()) return;
    // Pass device pointer to process function.
    sbp_state_set_io_context(state_.get(), device_.get());
    // Pass device read function to sbp_process.
    int result =
        sbp_process(state_.get(), &piksi_multi_cpp::Device::read_redirect);
    if (result < 0) {
      ROS_WARN_STREAM("Error sbp_process: " << result);
    }
  }
}

}  // namespace piksi_multi_cpp
