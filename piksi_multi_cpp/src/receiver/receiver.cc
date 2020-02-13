#include "piksi_multi_cpp/receiver/receiver.h"

#include <libsbp/sbp.h>
#include <ros/console.h>

namespace piksi_multi_cpp {

Receiver::Receiver(const Device::Ptr& device) : device_(device) {
  // Initialize SBP state.
  state_ = std::make_shared<sbp_state_t>();
  sbp_state_init(state_.get());
  // Pass device pointer to process function.
  sbp_state_set_io_context(state_.get(), device_.get());
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

bool Receiver::isRunning() const { return !thread_exit_requested_.load(); }

void Receiver::process() {
  // Setting thread_exit_requested_ will terminate the thread.
  while (!thread_exit_requested_.load()) {
    if (!device_.get()) {
      ROS_ERROR("Device pointer invalid.");
      thread_exit_requested_.store(true);
    }
    // Pass device read function to sbp_process.
    int result =
        sbp_process(state_.get(), &piksi_multi_cpp::Device::read_redirect);
    // Handle errors.
    if (result == SBP_READ_ERROR) {
      ROS_ERROR("Device reading error.");
      thread_exit_requested_.store(true);
    } else if (result < 0) {
      ROS_WARN_STREAM("Error sbp_process: " << result);
    }
  }
}

}  // namespace piksi_multi_cpp
