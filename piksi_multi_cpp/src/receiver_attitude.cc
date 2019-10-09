#include "piksi_multi_cpp/receiver_attitude.h"

namespace piksi_multi_cpp {

ReceiverAttitude::ReceiverAttitude(const ros::NodeHandle& nh,
                                   const std::shared_ptr<Device>& device)
    : Receiver(nh, device) {}

}  // namespace piksi_multi_cpp
