#ifndef PIKSI_MULTI_CPP_RECEIVER_POSITION_H_
#define PIKSI_MULTI_CPP_RECEIVER_POSITION_H_

#include <ros/ros.h>
#include <string>
#include "piksi_multi_cpp/device.h"
#include "piksi_multi_cpp/receiver.h"

namespace piksi_multi_cpp {

class ReceiverPosition : public Receiver {
 public:
  ReceiverPosition(const ros::NodeHandle& nh,
                      const std::shared_ptr<Device>& device);

 private:
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_RECEIVER_POSITION_H_
