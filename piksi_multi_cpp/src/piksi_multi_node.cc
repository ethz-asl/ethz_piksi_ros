#include <ros/ros.h>

#include "piksi_multi_cpp/ros_receiver.h"

using namespace piksi_multi_cpp;

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_multi");
  ros::NodeHandle nh_private("~");

  // Autodetect all receivers.
  auto receivers = ROSReceiver::createAllReceivers(nh_private);
  if (receivers.empty()) {
    ROS_FATAL("No receivers.");
    exit(1);
  }

  // Initialization
  for (auto rec : receivers) {
    if (!rec->init()) {
      ROS_FATAL("Error initializing receiver.");
      exit(1);
    }
  }

  // Process incoming data.
  // TODO(rikba): Multithreading!
  while (ros::ok()) {
    for (auto rec : receivers) rec->process();
    ros::spinOnce();
  }

  return 0;
}
