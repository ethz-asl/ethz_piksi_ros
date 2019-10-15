#ifndef PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_STATION_H_
#define PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_STATION_H_

#include <piksi_multi_cpp/observations/udp_observation_sender.h>
#include <ros/ros.h>
#include <string>
#include "piksi_multi_cpp/device/device.h"
#include "piksi_multi_cpp/receiver/receiver.h"

namespace piksi_multi_cpp {

class ReceiverBaseStation : public Receiver {
 public:
  ReceiverBaseStation(const ros::NodeHandle& nh,
                      const std::shared_ptr<Device>& device);

 private:
  std::shared_ptr<UDPObservationSender> udp_sender_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_STATION_H_
