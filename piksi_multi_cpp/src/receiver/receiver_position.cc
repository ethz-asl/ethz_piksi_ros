#include "piksi_multi_cpp/receiver/receiver_position.h"

namespace piksi_multi_cpp {

ReceiverPosition::ReceiverPosition(const ros::NodeHandle& nh,
                                   const std::shared_ptr<Device>& device)
    : Receiver(nh, device),
      // attach udp receiver with this class as consumer for remote corrections
      // received via UDP
      udp_receiver_(RawObservationInterface::Ptr(this)) {}

bool ReceiverPosition::init() {
  // do init of base class
  if (!Receiver::init()) {
    return false;
  }

  // start udp observation receiver.
  udp_receiver_.start(nh_.param("udp_observation_port", 26078));

  return true;
}

// Implementation of RawObservationInterface
void ReceiverPosition::insertObservation(
    const piksi_multi_cpp::RawObservation& data) {
  // forward observation data.
  device_->write(data);
}

}  // namespace piksi_multi_cpp
