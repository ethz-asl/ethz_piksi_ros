#ifndef PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_STATION_H_
#define PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_STATION_H_

#include <piksi_multi_cpp/observations/udp_observation_sender.h>
#include <ros/ros.h>
#include "piksi_multi_cpp/device/device.h"
#include "piksi_multi_cpp/receiver/receiver_ros.h"

namespace piksi_multi_cpp {

class ReceiverBaseStation : public ReceiverRos {
 public:
  ReceiverBaseStation(const ros::NodeHandle& nh, const Device::Ptr& device);

 private:
  void setupUDPSenders();
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_STATION_H_
