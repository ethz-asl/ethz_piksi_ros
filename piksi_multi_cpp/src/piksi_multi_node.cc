#include <ros/ros.h>

#include "piksi_multi_cpp/receiver/receiver.h"

using namespace piksi_multi_cpp;

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_multi");
  ros::NodeHandle nh_private("~");

  // Autodetect all receivers.
  auto receivers = Receiver::createAllReceivers(nh_private);
  if (receivers.empty()) {
    ROS_FATAL("No receivers.");
    exit(1);
  }

  // Start all receivers.
  for (auto rec : receivers) {
    if (!rec->init()) {
      ROS_FATAL("Error initializing receiver.");
      exit(1);
    }
  }

  ros::spin();

  return 0;
}
