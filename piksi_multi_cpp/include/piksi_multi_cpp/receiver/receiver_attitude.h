#ifndef PIKSI_MULTI_CPP_RECEIVER_ATTITUDE_H_
#define PIKSI_MULTI_CPP_RECEIVER_ATTITUDE_H_

#include <ros/ros.h>
#include <string>
#include "piksi_multi_cpp/device/device.h"
#include "piksi_multi_cpp/receiver/receiver.h"

// This class creates ROS publishers for attitude receivers. See also
// https://support.swiftnav.com/customer/en/portal/articles/2805901-piksi-multi---heading
namespace piksi_multi_cpp {

class ReceiverAttitude : public Receiver {
 public:
  ReceiverAttitude(const ros::NodeHandle& nh,
                   const std::shared_ptr<Device>& device);

 private:
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_ATTITUDE_H_
