#ifndef PIKSI_MULTI_ROS_BASE_STATION_H_
#define PIKSI_MULTI_ROS_BASE_STATION_H_

#include <ros/ros.h>
#include <string>
#include "piksi_multi_cpp/device.h"
#include "piksi_multi_cpp/ros_receiver.h"

namespace piksi_multi_cpp {

class ROSBaseStation : public ROSReceiver {
 public:
  ROSBaseStation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                 const std::shared_ptr<Device>& device, const std::string& ns);

 private:
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_ROS_BASE_STATION_H_
