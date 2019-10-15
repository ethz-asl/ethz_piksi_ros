#include "piksi_multi_cpp/receiver/receiver_base_station.h"
#include <piksi_multi_cpp/observations/udp_observation_sender.h>

namespace piksi_multi_cpp {

ReceiverBaseStation::ReceiverBaseStation(const ros::NodeHandle& nh,
                                         const std::shared_ptr<Device>& device)
    : Receiver(nh, device) {

  // Base station has a observation sender.
  obs_cbs_->addObservationCallbackListener(
      std::make_shared<UDPObservationSender>());
}

}  // namespace piksi_multi_cpp
