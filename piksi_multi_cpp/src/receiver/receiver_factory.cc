#include <ros/console.h>
#include "piksi_multi_cpp/device/device_factory.h"
#include "piksi_multi_cpp/receiver/receiver_attitude.h"
#include "piksi_multi_cpp/receiver/receiver_base_station.h"
#include "piksi_multi_cpp/receiver/receiver_factory.h"
#include "piksi_multi_cpp/receiver/receiver_position.h"
#include "piksi_multi_cpp/receiver/receiver_ros.h"
#include "piksi_multi_cpp/receiver/settings_io.h"

namespace piksi_multi_cpp {

Receiver::Ptr ReceiverFactory::createReceiverByReceiverType(
    const ros::NodeHandle& nh, const Device::Ptr& device,
    const ReceiverType type) {
  switch (type) {
    case ReceiverType::kBaseStationReceiver:
      return Receiver::Ptr(new ReceiverBaseStation(nh, device));
    case ReceiverType::kPositionReceiver:
      return Receiver::Ptr(new ReceiverPosition(nh, device));
    case ReceiverType::kAttitudeReceiver:
      return Receiver::Ptr(new ReceiverAttitude(nh, device));
    case ReceiverType::kSettingIo:
      return Receiver::Ptr(new SettingsIo(device));
    case ReceiverType::kUnknown:
      return Receiver::Ptr(new ReceiverRos(nh, device));
    case ReceiverType::kError:
      return nullptr;
    default:
      return nullptr;
  }
}

Receiver::Ptr ReceiverFactory::createReceiverByDevice(
    const ros::NodeHandle& nh, const Device::Ptr& device) {
  ReceiverType type = inferType(device);
  return createReceiverByReceiverType(nh, device, type);
}

std::vector<Receiver::Ptr>
ReceiverFactory::createAllReceiversByIdentifiersAndNaming(
    const ros::NodeHandle& nh, const Identifiers& ids) {
  // Create all devices.
  auto devices = DeviceFactory::createByIdentifiers(ids);

  // A counter variable to assign unique ids.
  std::map<ReceiverType, size_t> counter;
  // Create one ROS receiver per device.
  std::vector<std::shared_ptr<Receiver>> receivers;
  for (auto dev : devices) {
    ReceiverType type = inferType(dev);
    if (type == ReceiverType::kError) continue;
    // Initialize counter
    if (counter.find(type) == counter.end()) {
      // Initialize type counter.
      counter[type] = 0;
    }

    std::string ns = createNameSpace(type, counter[type]);
    ROS_INFO("Creating %s", ns.c_str());
    ros::NodeHandle nh_private(nh, ns);
    auto receiver = createReceiverByReceiverType(nh_private, dev, type);
    if (receiver.get()) receivers.push_back(receiver);
    // Increment type counter.
    counter[type] += 1;
  }

  ROS_WARN_COND(receivers.empty(), "No receiver created.");
  return receivers;
}

std::vector<SettingsIo::Ptr> ReceiverFactory::createSettingIoReceivers(
    const Identifiers& ids) {
  // Create all devices.
  auto devices = DeviceFactory::createByIdentifiers(ids);

  // Create all receivers.
  std::vector<std::shared_ptr<SettingsIo>> receivers;
  for (auto dev : devices) {
    auto settings_io = std::make_shared<SettingsIo>(dev);
    if (settings_io.get()) receivers.push_back(settings_io);
  }

  ROS_WARN_COND(receivers.empty(), "No settings_io receivers created.");
  return receivers;
}

std::string ReceiverFactory::createNameSpace(const ReceiverType type,
                                             const size_t id) {
  std::string type_name = "";
  switch (type) {
    case ReceiverType::kBaseStationReceiver:
      type_name = "base_station_receiver";
      break;
    case ReceiverType::kPositionReceiver:
      type_name = "position_receiver";
      break;
    case ReceiverType::kAttitudeReceiver:
      type_name = "attitude_receiver";
      break;
    case ReceiverType::kUnknown:
      type_name = "unknown_receiver";
      break;
    default:
      type_name = "default";
  }
  return type_name + "_" + std::to_string(id);
}

ReceiverFactory::ReceiverType ReceiverFactory::inferType(
    const Device::Ptr& dev) {
  if (!dev.get()) return ReceiverType::kError;

  // Create settings object.
  SettingsIo settings_io(dev);
  if (!settings_io.init()) return ReceiverType::kError;

  // Identify base station.
  while (!settings_io.readSetting("surveyed_position", "broadcast")) {
  }
  if (settings_io.checkBoolTrue()) return ReceiverType::kBaseStationReceiver;
  // Identify attitude receiver.
  while (!settings_io.readSetting("solution", "send_heading")) {
  }
  if (settings_io.checkBoolTrue())
    return ReceiverType::kAttitudeReceiver;
  else
    return ReceiverType::kPositionReceiver;

  ROS_WARN("Cannot infer receiver type.");
  return ReceiverType::kUnknown;
}

}  // namespace piksi_multi_cpp
