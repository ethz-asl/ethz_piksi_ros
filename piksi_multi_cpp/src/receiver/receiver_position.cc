#include "piksi_multi_cpp/receiver/receiver_position.h"

namespace piksi_multi_cpp {

ReceiverPosition::ReceiverPosition(const ros::NodeHandle& nh,
                                   const Device::Ptr& device)
    : ReceiverRos(nh, device),
      // attach udp receiver with this class as consumer for remote corrections
      // received via UDP
      udp_receiver_(RawObservationInterface::Ptr(this)) {}

bool ReceiverPosition::init() {
  // do init of base class
  if (!ReceiverRos::init()) {
    return false;
  }

  // start udp observation receiver.
  ros::NodeHandle nh_private("~");
  udp_receiver_.start(nh_private.param("udp_observation_port", 26078));

  return true;
}

// Implementation of RawObservationInterface
void ReceiverPosition::insertObservation(
    const piksi_multi_cpp::RawObservation& data) {
  // forward observation data.
  device_->write(data);
}

}  // namespace piksi_multi_cpp
