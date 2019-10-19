#include "piksi_multi_cpp/receiver/receiver_base_station.h"
#include <piksi_multi_cpp/observations/udp_observation_sender.h>
#include <boost/algorithm/string.hpp>

namespace piksi_multi_cpp {

ReceiverBaseStation::ReceiverBaseStation(const ros::NodeHandle& nh,
                                         const std::shared_ptr<Device>& device)
    : Receiver(nh, device) {
  setupUDPSenders();
}

void ReceiverBaseStation::setupUDPSenders() {
  auto udp_port = nh_.param<int>("udp_port", 26078);

  std::vector<std::string> udp_interfaces =
      getVectorParam("udp_broadcast_interfaces");
  for (const auto& interface : udp_interfaces) {
    auto udp_sender_ = UDPObservationSender::toNetwork(interface, udp_port);
    if (udp_sender_) {
      udp_sender_->open();
      // Register with observation callbacks
      obs_cbs_->addObservationCallbackListener(
          CBtoRawObsConverter::createFor(udp_sender_));
    }
  }

  // set up unicasts
  std::vector<std::string> udp_unicasts = getVectorParam("udp_unicast_targets");

  for (const auto& unicast : udp_unicasts) {
    auto udp_sender_ = UDPObservationSender::toHost(unicast, udp_port);
    if (udp_sender_) {
      udp_sender_->open();

      // Register with observation callbacks
      obs_cbs_->addObservationCallbackListener(
          CBtoRawObsConverter::createFor(udp_sender_));
    }
  }
}

}  // namespace piksi_multi_cpp
