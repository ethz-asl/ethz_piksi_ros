#include <ros/ros.h>

#include "piksi_multi_cpp/piksi_multi.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_multi");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  int poll_rate = nh_private.param<int>("poll_rate", 100);
  piksi_multi_cpp::PiksiMulti driver(nh, nh_private);

  if (driver.open()) {
    ROS_INFO("Port(s) opened.");
  } else {
    ROS_FATAL("Error opening port(s).");
    exit(1);
  }

  ros::Rate r(poll_rate);
  while (ros::ok()) {
    driver.read();
    ros::spinOnce();
    r.sleep();
  }

  if (driver.close()) {
    ROS_INFO("Port(s) closed.");
  } else {
    ROS_FATAL("Error closing port(s).");
    exit(1);
  }

  return 0;
}
