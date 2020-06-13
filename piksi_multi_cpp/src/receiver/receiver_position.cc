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

  // Start file logger if requested
  auto log_to_file = nh_private.param<bool>("log_observations_to_file", false);
  if (log_to_file) {
    auto log_file_dir =
        nh_private.param<std::string>("log_file_dir", "/tmp/tmp_observations.sbp");
    ReceiverRos::startFileLogger(log_file_dir);
  }

  return true;
}

// Implementation of RawObservationInterface
void ReceiverPosition::insertObservation(
    const piksi_multi_cpp::RawObservation& data) {
  // forward observation data.
  device_->write(data);
}

}  // namespace piksi_multi_cpp
