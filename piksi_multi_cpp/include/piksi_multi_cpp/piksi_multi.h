#ifndef PIKSI_MULTI_CPP_PIKSI_MULTI_H_
#define PIKSI_MULTI_CPP_PIKSI_MULTI_H_

#include <ros/ros.h>
#include "piksi_multi_cpp/device_usb.h"

namespace piksi_multi_cpp {

class PiksiMulti {
 public:
  PiksiMulti(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool open();
  bool close();
  bool read();

 private:
  void getROSParameters();
  void advertiseTopics();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  struct Parameters {
    Parameters() {}
  };

  Parameters params_;

  std::vector<DeviceUSB> devices_usb_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_PIKSI_MULTI_H_
