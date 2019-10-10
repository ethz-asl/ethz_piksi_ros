#include "piksi_multi_cpp/receiver/receiver.h"

#include <libsbp/sbp.h>
#include <piksi_rtk_msgs/Heartbeat.h>
#include "piksi_multi_cpp/device/device.h"

// Forward declarations
#include "piksi_multi_cpp/receiver/receiver_attitude.h"
#include "piksi_multi_cpp/receiver/receiver_base_station.h"
#include "piksi_multi_cpp/receiver/receiver_position.h"

// SBP message definitions.
#include <libsbp/system.h>

namespace piksi_multi_cpp {

std::vector<Receiver::ReceiverType> Receiver::kTypeVec =
    std::vector<Receiver::ReceiverType>(
        {kBaseStationReceiver, kPositionReceiver, kAttitudeReceiver, kUnknown});

Receiver::Receiver(const ros::NodeHandle& nh,
                   const std::shared_ptr<Device>& device)
    : nh_(nh), device_(device), is_running_(true) {
  // Initialize SBP state.
  state_ = std::make_shared<sbp_state_t>();
  sbp_state_init(state_.get());

  // Register callbacks.
  cb_.push_back(SBPCallback::create(nh, SBP_MSG_HEARTBEAT, state_));
}

std::shared_ptr<Receiver> Receiver::create(
    const ros::NodeHandle& nh, const std::shared_ptr<Device>& device,
    const ReceiverType type) {
  switch (type) {
    case ReceiverType::kBaseStationReceiver:
      return std::shared_ptr<Receiver>(new ReceiverBaseStation(nh, device));
    case ReceiverType::kPositionReceiver:
      return std::shared_ptr<Receiver>(new ReceiverPosition(nh, device));
    case ReceiverType::kAttitudeReceiver:
      return std::shared_ptr<Receiver>(new ReceiverAttitude(nh, device));
    case ReceiverType::kUnknown:
      return std::shared_ptr<Receiver>(new Receiver(nh, device));
    default:
      return nullptr;
  }
}

std::shared_ptr<Receiver> Receiver::create(
    const ros::NodeHandle& nh, const std::shared_ptr<Device>& device) {
  ReceiverType type = inferType(device);
  return create(nh, device, type);
}

Receiver::~Receiver() {
  // Close thread.
  is_running_.store(false);
  process_thread_.join();

  if (device_) device_->close();
}

bool Receiver::init() {
  if (!device_.get()) {
    ROS_ERROR("Device not set.");
    return false;
  }

  // Open attached device.
  if (!device_->open()) {
    ROS_ERROR("Cannot open device.");
    return false;
  }

  process_thread_ = std::thread(&Receiver::process, this);

  return true;
}

void Receiver::process() {
  while (is_running_.load()) {
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
}

std::string Receiver::createNameSpace(const ReceiverType type,
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

std::vector<std::shared_ptr<Receiver>> Receiver::createAllReceivers(
    const ros::NodeHandle& nh) {
  // Create all devices.
  std::vector<std::shared_ptr<Device>> devices = Device::createAllDevices();

  // A counter variable to assign unique ids.
  std::map<ReceiverType, size_t> counter;
  for (const auto type : kTypeVec) counter[type] = 0;

  // Create one ROS receiver per device.
  std::vector<std::shared_ptr<Receiver>> receivers;
  for (auto dev : devices) {
    ReceiverType type = inferType(dev);
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

Receiver::ReceiverType Receiver::inferType(const std::shared_ptr<Device>& dev) {
  if (!dev.get()) return ReceiverType::kUnknown;

  ROS_WARN("inferType not implemented.");
  return ReceiverType::kUnknown;
}

}  // namespace piksi_multi_cpp
