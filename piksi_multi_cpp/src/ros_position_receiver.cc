#include "piksi_multi_cpp/ros_position_receiver.h"

namespace piksi_multi_cpp {

ROSPositionReceiver::ROSPositionReceiver(const ros::NodeHandle& nh,
                                         const ros::NodeHandle& nh_private,
                                         const std::shared_ptr<Device>& device,
                                         const std::string& ns)
    : ROSReceiver(nh, nh_private, device, ns) {}

}  // namespace piksi_multi_cpp
