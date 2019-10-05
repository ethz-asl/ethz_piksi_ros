#include <ros/ros.h>

#include "piksi_multi_cpp/piksi_multi.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_multi");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  piksi_multi_cpp::PiksiMulti piksi_multi(nh, nh_private);

  piksi_multi_cpp::PiksiMulti driver(nh, nh_private);

  if (driver.connect()) {
    ROS_INFO("Port opened.");
  } else {
    ROS_FATAL("Error opening port.");
    exit(1);
  }

  while (ros::ok()) {
    if(!driver.read()) {
      ROS_WARN("Failed reading data.");
    }
    ros::spinOnce();
  }

  if (driver.disconnect()) {
    ROS_INFO("Port closed.");
  } else {
    ROS_FATAL("Error closing port.");
    exit(1);
  }

  return 0;
}
