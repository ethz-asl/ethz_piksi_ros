#include "piksi_multi_cpp/receiver_base_station.h"

namespace piksi_multi_cpp {

ReceiverBaseStation::ReceiverBaseStation(
    const ros::NodeHandle& nh, const std::shared_ptr<Device>& device)
    : Receiver(nh, device) {}

}  // namespace piksi_multi_cpp
