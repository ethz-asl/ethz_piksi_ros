#include "piksi_multi_cpp/receiver/receiver_position.h"

namespace piksi_multi_cpp {

ReceiverPosition::ReceiverPosition(const ros::NodeHandle& nh,
                                   const std::shared_ptr<Device>& device)
    : Receiver(nh, device) {}

}  // namespace piksi_multi_cpp
