#include "piksi_multi_cpp/ros_position_receiver.h"

namespace piksi_multi_cpp {

ROSPositionReceiver::ROSPositionReceiver(const ros::NodeHandle& nh,
                                         const std::shared_ptr<Device>& device)
    : ROSReceiver(nh, device) {}

}  // namespace piksi_multi_cpp
