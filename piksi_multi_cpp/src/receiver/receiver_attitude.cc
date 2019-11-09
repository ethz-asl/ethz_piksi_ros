#include "piksi_multi_cpp/receiver/receiver_attitude.h"

namespace piksi_multi_cpp {

ReceiverAttitude::ReceiverAttitude(const ros::NodeHandle& nh,
                                   const Device::Ptr& device)
    : ReceiverRos(nh, device) {}

}  // namespace piksi_multi_cpp
