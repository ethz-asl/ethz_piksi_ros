#ifndef PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_POSITION_H_
#define PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_POSITION_H_

#include <piksi_multi_cpp/device/device.h>
#include <piksi_multi_cpp/observations/raw_observation_interface.h>
#include <piksi_multi_cpp/observations/udp_observation_receiver.h>
#include <ros/ros.h>
#include "piksi_multi_cpp/receiver/receiver_ros.h"

namespace piksi_multi_cpp {

class ReceiverPosition : public ReceiverRos, public RawObservationInterface {
 public:
  ReceiverPosition(const ros::NodeHandle& nh, const Device::Ptr& device);

  bool init() override;

  // From RawObservationsConsumer interface
  virtual void insertObservation(const RawObservation& data);

 protected:
  UDPObservationReceiver udp_receiver_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_POSITION_H_
