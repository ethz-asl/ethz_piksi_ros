#include <ros/ros.h>

#include <boost/algorithm/string.hpp>
#include "piksi_multi_cpp/receiver/receiver_factory.h"
using namespace piksi_multi_cpp;

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_multi");
  ros::NodeHandle nh_private("~");

  // By default, autodetect all usb devices.
  auto device_ids = nh_private.param<std::string>("device_ids", "usb://*");

  // split device_ids by ";" to get multiple identifiers
  Identifiers ids;
  boost::algorithm::split(ids, device_ids, boost::is_any_of(";"));

  auto receivers = ReceiverFactory::createAllReceiversByIdentifiersAndNaming(
      nh_private, ids);
  if (receivers.empty()) {
    ROS_FATAL("No receivers.");
    exit(1);
  }

  // Start all receivers.
  for (auto rec : receivers) {
    if (!rec->init()) {
      ROS_FATAL("Error initializing receiver");
    }
  }

  // Watchdog to count running receivers.
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    // check how many are running.
    int running_receivers = std::count_if(
        receivers.begin(), receivers.end(),
        [](auto receiver) { return receiver.get() && receiver->isRunning(); });

    if (running_receivers < 1) {
      ROS_FATAL("No receivers initialized. Stopping.");
      exit(1);
    } else {
      ROS_INFO_STREAM_ONCE("Found and initialized " << running_receivers
                                                    << " Receivers");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
