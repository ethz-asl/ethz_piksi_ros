#ifndef PIKSI_MULTI_CPP_RECEIVER_RECEIVER_H_
#define PIKSI_MULTI_CPP_RECEIVER_RECEIVER_H_

#include <libsbp/sbp.h>
#include <atomic>
#include <memory>
#include <thread>
#include "piksi_multi_cpp/device/device.h"

namespace piksi_multi_cpp {

// This class offers a (ROS) interface for Piksi Multi receivers.
class Receiver {
 public:
  typedef std::shared_ptr<Receiver> Ptr;

  Receiver(const Device::Ptr& device);

  // Closes device.
  ~Receiver();

  // Open device.
  virtual bool init();

  // Checks if thread is running
  bool isRunning() const;

 protected:
  // The actual hardware interface.
  Device::Ptr device_;

 protected:
  // The sbp state.
  std::shared_ptr<sbp_state_t> state_;

 private:
  // Read device and process SBP callbacks.
  void process();

  // Start thread that reads device and processes SBP messages. This thread is
  // terminated when the is_running flag ist set false during object
  // destruction.
  std::thread process_thread_;
  std::atomic_bool thread_exit_requested_ = false;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_RECEIVER_H_
