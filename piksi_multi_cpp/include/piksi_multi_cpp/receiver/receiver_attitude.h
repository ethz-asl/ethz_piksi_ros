#ifndef PIKSI_MULTI_CPP_RECEIVER_RECEIVER_ATTITUDE_H_
#define PIKSI_MULTI_CPP_RECEIVER_RECEIVER_ATTITUDE_H_

#include <ros/ros.h>
#include "piksi_multi_cpp/device/device.h"
#include "piksi_multi_cpp/receiver/receiver_ros.h"

// This class creates ROS publishers for attitude receivers. See also
// https://support.swiftnav.com/customer/en/portal/articles/2805901-piksi-multi---heading
namespace piksi_multi_cpp {

class ReceiverAttitude : public ReceiverRos {
 public:
  ReceiverAttitude(const ros::NodeHandle& nh, const Device::Ptr& device);

 private:
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_RECEIVER_ATTITUDE_H_
