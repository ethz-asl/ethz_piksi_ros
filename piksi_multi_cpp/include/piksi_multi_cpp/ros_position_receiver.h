#ifndef PIKSI_MULTI_ROS_POSITION_RECEIVER_H_
#define PIKSI_MULTI_ROS_POSITION_RECEIVER_H_

#include <ros/ros.h>
#include <string>
#include "piksi_multi_cpp/device.h"
#include "piksi_multi_cpp/ros_receiver.h"

namespace piksi_multi_cpp {

class ROSPositionReceiver : public ROSReceiver {
 public:
  ROSPositionReceiver(const ros::NodeHandle& nh,
                      const std::shared_ptr<Device>& device);

 private:
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_ROS_POSITION_RECEIVER_H_
