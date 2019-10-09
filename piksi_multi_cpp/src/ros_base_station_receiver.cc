#include "piksi_multi_cpp/ros_base_station_receiver.h"

namespace piksi_multi_cpp {

ROSBaseStationReceiver::ROSBaseStationReceiver(
    const ros::NodeHandle& nh, const std::shared_ptr<Device>& device)
    : ROSReceiver(nh, device) {}

}  // namespace piksi_multi_cpp
