#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <memory>
#include <set>
#include <string>
#include "piksi_multi_cpp/receiver/receiver_factory.h"
#include "piksi_multi_cpp/receiver/settings_io.h"

using namespace piksi_multi_cpp;

int main(int argc, char** argv) {
  ros::init(argc, argv, "piksi_multi_config");
  ros::NodeHandle nh_private("~");

  // By default, autodetect all usb devices.
  auto device_ids = nh_private.param<std::string>("device_ids", "usb://*");

  // split device_ids by ";" to get multiple identifiers
  Identifiers ids;
  boost::algorithm::split(ids, device_ids, boost::is_any_of(";"));

  // Create SettingIo receivers.
  auto receivers = ReceiverFactory::createSettingIoReceivers(nh_private, ids);
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
  // Get config type.
  std::string config_type =
      nh_private.param<std::string>("config_type", "rover");
  std::set<std::string> config_types{"att", "base", "ref", "rover"};
  if (config_types.count(config_type) < 0) {
    ROS_FATAL("Config type '%s' does not exist.", config_type);
    exit(1);
  }
  std::string pkg_path = ros::package::getPath("piksi_multi_cpp");

  // std::string
  //    config_file = pkg_path + "/config_" + config_type + "";  // =
  //    nh_private.param<std::string>("config_file", "");
  for (auto rec : receivers) {
    auto setting_io = std::static_pointer_cast<SettingsIo>(rec);

    // Get config file.
    if (!setting_io->readSetting("system_info", "firmware_version")) {
      ROS_FATAL("Cannot read firmware version.");
      continue;
    }
    auto firmware_version = setting_io->getValue();
    ROS_INFO("%s", firmware_version.c_str());
    std::string config_file;
    if (!setting_io->openConfig(config_file)) {
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
