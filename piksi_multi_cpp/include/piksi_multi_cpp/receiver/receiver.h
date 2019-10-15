#ifndef PIKSI_MULTI_CPP_RECEIVER_RECEIVER_H_
#define PIKSI_MULTI_CPP_RECEIVER_RECEIVER_H_

#include <libsbp/sbp.h>
#include <ros/ros.h>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "piksi_multi_cpp/device/device.h"
#include "piksi_multi_cpp/receiver/receiver.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include <piksi_multi_cpp/sbp_callback_handler/sbp_observation_callback_handler.h>

namespace piksi_multi_cpp {

// This class offers a (ROS) interface for Piksi Multi receivers.
class Receiver {
 public:
  typedef std::shared_ptr<Receiver> Ptr;

  Receiver(const ros::NodeHandle& nh, const Device::Ptr& device);

  // Closes device.
  ~Receiver();

  // Open device.
  virtual bool init();

  // Checks if thread is running
  bool isRunning() const;

 protected:
  // ROS node handle in the correct receiver namespace.
  ros::NodeHandle nh_;
  // The actual hardware interface.
  Device::Ptr device_;

  // Observation callbackhandlers
  std::unique_ptr<SBPObservationCallbackHandler> obs_cbs_;

 private:
  // Read device and process SBP callbacks.
  void process();
  // Start thread that reads device and processes SBP messages. This thread is
  // terminated when the is_running flag ist set false during object
  // destruction.
  std::thread process_thread_;
  std::atomic_bool thread_exit_requested_;

  // The sbp state.
  std::shared_ptr<sbp_state_t> state_;

  // Relaying all SBP messages. Common for all receivers.
  std::vector<SBPCallbackHandler::Ptr> relay_cbs_;


};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_RECEIVER_H_
