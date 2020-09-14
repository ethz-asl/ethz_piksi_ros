#ifndef PIKSI_MULTI_CPP_RECEIVER_RECEIVER_ROS_H_
#define PIKSI_MULTI_CPP_RECEIVER_RECEIVER_ROS_H_

#include <ros/ros.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "piksi_multi_cpp/receiver/settings_io.h"
#include "piksi_multi_cpp/sbp_callback_handler/geotf_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/position_sampler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_ephemeris_callback_handler.h"
#include "piksi_multi_cpp/sbp_callback_handler/sbp_observation_callback_handler.h"

namespace piksi_multi_cpp {

// This class offers a (ROS) interface for Piksi Multi receivers.
class ReceiverRos : public SettingsIo {
 public:
  typedef std::shared_ptr<ReceiverRos> Ptr;

  ReceiverRos(const ros::NodeHandle& nh, const Device::Ptr& device);

  // set up user ID
  bool init() override;

  bool startFileLogger();

 protected:
  // ROS node handle in the correct receiver namespace.
  ros::NodeHandle nh_;

  // Sender ID of device
  uint16_t sbp_sender_id_{0x42};

  // Observation & Ephemeris callbackhandlers
  std::unique_ptr<SBPObservationCallbackHandler> obs_cbs_;
  std::unique_ptr<SBPEphemerisCallbackHandler> eph_cbs_;

  // get vector valued string params
  std::vector<std::string> getVectorParam(
      const std::string& name, const std::string& default_value = "");

  // Manages geotf transformations.
  GeoTfHandler::Ptr geotf_handler_;
  // Averages the position over multiple ECEF messages.
  PositionSampler::Ptr position_sampler_;

 private:
  // Relaying all SBP messages. Common for all receivers.
  std::vector<SBPCallbackHandler::Ptr> sbp_relays_;
  // Relaying all ROS messages. Common for all receivers.
  std::vector<SBPCallbackHandler::Ptr> ros_relays_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_RECEIVER_ROS_H_
