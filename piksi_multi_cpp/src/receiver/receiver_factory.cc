#include <ros/console.h>
#include "piksi_multi_cpp/device/device_factory.h"
#include "piksi_multi_cpp/receiver/receiver.h"
#include "piksi_multi_cpp/receiver/receiver_attitude.h"
#include "piksi_multi_cpp/receiver/receiver_base_station.h"
#include "piksi_multi_cpp/receiver/receiver_factory.h"
#include "piksi_multi_cpp/receiver/receiver_position.h"

namespace piksi_multi_cpp {

Receiver::ReceiverPtr
ReceiverFactory::createReceiverByNodeHandleDeviceAndReceiverType(
    const ros::NodeHandle& nh, const Device::DevicePtr& device,
    const Receiver::ReceiverType type) {
  switch (type) {
    case Receiver::ReceiverType::kBaseStationReceiver:
      return Receiver::ReceiverPtr(new ReceiverBaseStation(nh, device));
    case Receiver::ReceiverType::kPositionReceiver:
      return Receiver::ReceiverPtr(new ReceiverPosition(nh, device));
    case Receiver::ReceiverType::kAttitudeReceiver:
      return Receiver::ReceiverPtr(new ReceiverAttitude(nh, device));
    case Receiver::ReceiverType::kUnknown:
      return Receiver::ReceiverPtr(new Receiver(nh, device));
    default:
      return nullptr;
  }
}

Receiver::ReceiverPtr ReceiverFactory::createReceiverByNodeHandleAndDevice(
    const ros::NodeHandle& nh, const Device::DevicePtr& device) {
  Receiver::ReceiverType type = inferType(device);
  return createReceiverByNodeHandleDeviceAndReceiverType(nh, device, type);
}

std::vector<Receiver::ReceiverPtr>
ReceiverFactory::createAllReceiversByAutoDiscoveryAndNaming(
    const ros::NodeHandle& nh) {
  // Create all devices.
  auto devices = DeviceFactory::createAllDevicesByAutodiscovery();

  // A counter variable to assign unique ids.
  std::map<Receiver::ReceiverType, size_t> counter;
  for (const auto type : Receiver::kTypeVec) counter[type] = 0;

  // Create one ROS receiver per device.
  std::vector<std::shared_ptr<Receiver>> receivers;
  for (auto dev : devices) {
    Receiver::ReceiverType type = inferType(dev);
    size_t unique_id = counter[type];
    std::string ns = createNameSpace(type, unique_id);
    ros::NodeHandle nh_private(nh, ns);
    auto receiver =
        createReceiverByNodeHandleDeviceAndReceiverType(nh_private, dev, type);
    if (receiver.get()) receivers.push_back(receiver);
    counter[type] = counter[type] + 1;
  }

  ROS_WARN_COND(receivers.empty(), "No receiver created.");
  return receivers;
}

std::string ReceiverFactory::createNameSpace(const Receiver::ReceiverType type,
                                             const size_t id) {
  std::string type_name = "";
  switch (type) {
    case Receiver::ReceiverType::kBaseStationReceiver:
      type_name = "base_station_receiver";
      break;
    case Receiver::ReceiverType::kPositionReceiver:
      type_name = "position_receiver";
      break;
    case Receiver::ReceiverType::kAttitudeReceiver:
      type_name = "attitude_receiver";
      break;
    case Receiver::ReceiverType::kUnknown:
      type_name = "unknown_receiver";
      break;
    default:
      type_name = "default";
  }
  return type_name + "_" + std::to_string(id);
}

Receiver::ReceiverType ReceiverFactory::inferType(
    const Device::DevicePtr& dev) {
  if (!dev.get()) return Receiver::ReceiverType::kUnknown;

  ROS_WARN("inferType not implemented.");
  return Receiver::ReceiverType::kUnknown;
}

}  // namespace piksi_multi_cpp
