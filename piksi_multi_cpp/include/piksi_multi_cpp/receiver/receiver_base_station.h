#ifndef PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_STATION_H_
#define PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_STATION_H_

#include <piksi_multi_cpp/observations/udp_observation_sender.h>
#include <piksi_rtk_msgs/PositionWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "piksi_multi_cpp/device/device.h"
#include "piksi_multi_cpp/receiver/receiver_ros.h"

namespace piksi_multi_cpp {

class ReceiverBaseStation : public ReceiverRos {
 public:
  ReceiverBaseStation(const ros::NodeHandle& nh, const Device::Ptr& device);

 private:
  void setupUDPSenders();
  bool overwriteBasePositionCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res);
  void sampledPositionCallback(
      const piksi_rtk_msgs::PositionWithCovarianceStamped::Ptr& msg);
  void setupBaseStationSampling();

  ros::ServiceServer overwrite_base_position_srv_;
  ros::Subscriber ml_estimate_sub_;
  ros::Subscriber receiver_state_sub_;
  bool wait_for_sampled_position_ = true;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_RECEIVER_BASE_STATION_H_
