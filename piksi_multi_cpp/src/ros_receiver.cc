#include "piksi_multi_cpp/ros_receiver.h"

#include <libsbp/sbp.h>
#include <piksi_rtk_msgs/Heartbeat.h>
#include "piksi_multi_cpp/device.h"

// Forward declarations
#include "piksi_multi_cpp/ros_attitude_receiver.h"
#include "piksi_multi_cpp/ros_base_station_receiver.h"
#include "piksi_multi_cpp/ros_position_receiver.h"

namespace piksi_multi_cpp {

std::vector<ROSReceiver::Type> ROSReceiver::kTypeVec =
    std::vector<ROSReceiver::Type>(
        {kBaseStationReceiver, kPositionReceiver, kAttitudeReceiver, kUnknown});

ROSReceiver::ROSReceiver(const ros::NodeHandle& nh,
                         const std::shared_ptr<Device>& device)
    : nh_(nh), device_(device) {
  // Initialize SBP state.
  state_ = std::make_shared<sbp_state_t>();
  sbp_state_init(state_.get());
}

std::shared_ptr<ROSReceiver> ROSReceiver::create(
    const ros::NodeHandle& nh, const std::shared_ptr<Device>& device,
    const Type type) {
  switch (type) {
    case Type::kBaseStationReceiver:
      return std::shared_ptr<ROSReceiver>(
          new ROSBaseStationReceiver(nh, device));
    case Type::kPositionReceiver:
      return std::shared_ptr<ROSReceiver>(new ROSPositionReceiver(nh, device));
    case Type::kAttitudeReceiver:
      return std::shared_ptr<ROSReceiver>(new ROSAttitudeReceiver(nh, device));
    case Type::kUnknown:
      return std::shared_ptr<ROSReceiver>(new ROSReceiver(nh, device));
    default:
      return nullptr;
  }
}

std::shared_ptr<ROSReceiver> ROSReceiver::create(
    const ros::NodeHandle& nh, const std::shared_ptr<Device>& device) {
  Type type = inferType(device);
  return create(nh, device, type);
}

ROSReceiver::~ROSReceiver() {
  if (device_) device_->close();
}

bool ROSReceiver::init() {
  if (!device_.get()) {
    ROS_ERROR("Device not set.");
    return false;
  }
  return device_->open();
}

void ROSReceiver::process() {
  if (!device_.get()) return;
  // Pass device pointer to process function.
  sbp_state_set_io_context(state_.get(), device_.get());
  // Pass device read function to sbp_process.
  int result =
      sbp_process(state_.get(), &piksi_multi_cpp::Device::read_redirect);
  if (result < 0) {
    ROS_WARN_STREAM("Error sbp_process: " << result);
  }
}

std::string ROSReceiver::createNameSpace(const Type type, const size_t id) {
  std::string type_name = "";
  switch (type) {
    case Type::kBaseStationReceiver:
      type_name = "base_station_receiver";
      break;
    case Type::kPositionReceiver:
      type_name = "position_receiver";
      break;
    case Type::kAttitudeReceiver:
      type_name = "attitude_receiver";
      break;
    case Type::kUnknown:
      type_name = "unknown_receiver";
    default:
      type_name = "default";
  }
  return type_name + "_" + std::to_string(id);
}

std::vector<std::shared_ptr<ROSReceiver>> ROSReceiver::createAllReceivers(
    const ros::NodeHandle& nh) {
  // Create all devices.
  std::vector<std::shared_ptr<Device>> devices = Device::createAllDevices();

  // A counter variable to assign unique ids.
  std::map<Type, size_t> counter;
  for (const auto type : kTypeVec) counter[type] = 0;

  // Create one ROS receiver per device.
  std::vector<std::shared_ptr<ROSReceiver>> receivers;
  for (auto dev : devices) {
    Type type = inferType(dev);
    size_t unique_id = counter[type];
    std::string ns = createNameSpace(type, unique_id);
    ros::NodeHandle nh_private(nh, ns);
    auto receiver = create(nh_private, dev, type);
    if (receiver.get()) receivers.push_back(receiver);
    counter[type] = counter[type] + 1;
  }

  ROS_WARN_COND(receivers.empty(), "No receiver created.");
  return receivers;
}

ROSReceiver::Type ROSReceiver::inferType(const std::shared_ptr<Device>& dev) {
  if (!dev.get()) return Type::kUnknown;

  ROS_WARN("inferType not implemented.");
  return Type::kUnknown;
}

}  // namespace piksi_multi_cpp
