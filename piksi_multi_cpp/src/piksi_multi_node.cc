#include <ros/ros.h>

#include "piksi_multi_cpp/piksi_multi.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_multi");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  piksi_multi_cpp::PiksiMulti piksi_multi(nh, nh_private);
  ros::spin();
  return 0;
}
