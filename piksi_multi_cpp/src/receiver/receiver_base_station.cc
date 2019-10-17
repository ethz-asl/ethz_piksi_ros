#include "piksi_multi_cpp/receiver/receiver_base_station.h"
#include <piksi_multi_cpp/observations/udp_observation_sender.h>

namespace piksi_multi_cpp {

ReceiverBaseStation::ReceiverBaseStation(const ros::NodeHandle& nh,
                                         const std::shared_ptr<Device>& device)
    : Receiver(nh, device) {
  // udp_sender_ = UDPObservationSender::toNetwork("enx3ce1a1257601");
  udp_sender_ = UDPObservationSender::toHost("129.132.39.59");
  if (udp_sender_) {
    udp_sender_->open();

    // Base station has a observation sender.
    obs_cbs_->addObservationCallbackListener(
        CBtoRawObsConverter::createFor(udp_sender_));
  }
}

}  // namespace piksi_multi_cpp
