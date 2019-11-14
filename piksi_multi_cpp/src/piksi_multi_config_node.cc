#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <experimental/filesystem>
#include <memory>
#include <set>
#include <string>
#include "piksi_multi_cpp/receiver/receiver_factory.h"
#include "piksi_multi_cpp/receiver/settings_io.h"

using namespace piksi_multi_cpp;

namespace fs = std::experimental::filesystem;

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_multi_config");
  ros::NodeHandle nh_private("~");

  // Get rosparams.
  // By default, autodetect all usb devices.
  auto device_ids = nh_private.param<std::string>("device_ids", "usb://*");
  // Get config type.
  std::string config_type = nh_private.param<std::string>(
      "config_type", "rover");  // Possible types: "att", "base", "ref", "rover"

  // split device_ids by ";" to get multiple identifiers
  Identifiers ids;
  boost::algorithm::split(ids, device_ids, boost::is_any_of(";"));

  // Create SettingIo receivers.
  auto receivers = ReceiverFactory::createSettingIoReceivers(ids);
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

  // Open configs.
  // Search config file.
  std::string pkg_path = ros::package::getPath("piksi_multi_cpp");
  std::string config_file;
  for (const auto& entry : fs::directory_iterator(pkg_path + "/cfg/")) {
    if (entry.path().extension().compare(".ini") != 0) continue;
    if (entry.path().stem().string().find(config_type) != std::string::npos) {
      config_file = entry.path().string();
      break;
    }
  }
  // Overwrite config file with rosparam.
  config_file = nh_private.param<std::string>("config_file", config_file);

  // Now load parameters.
  for (auto rec : receivers) {
    // Process config file.
    if (!rec->updateConfig(config_file)) {
      ROS_FATAL("Error opening config.");
    }
  }

  // check how many are running.
  int running_receivers = std::count_if(
      receivers.begin(), receivers.end(),
      [](auto receiver) { return receiver.get() && receiver->isRunning(); });

  if (running_receivers < 1) {
    ROS_FATAL("No receivers initialized. stopping.");
    exit(1);
  } else {
    ROS_INFO_STREAM("Found and initialized " << running_receivers
                                             << " Receivers");
  }

  ros::spinOnce();

  return 0;
}
