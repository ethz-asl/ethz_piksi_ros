
#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATION_SENDER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATION_SENDER_H_
#include <netinet/in.h>
#include <piksi_multi_cpp/observations/cb_to_raw_obs_converter.h>

namespace piksi_multi_cpp {

/*
 * Class that forwards raw observations via UDP. To be used by the base station.
 *
 * Does consume raw observations for forwarding and thus has
 * RawObservationsInterface.
 */
class UDPObservationSender : public RawObservationInterface {
 public:
  // Factory method for regular unicast
  static std::shared_ptr<UDPObservationSender> toHost(
      const std::string& host_ip, const uint port = 26078);

  // Factory method for network broadcast for a specific interface
  static std::shared_ptr<UDPObservationSender> toNetwork(
      const std::string& interface_name, const uint port = 26078);

  // Factory method for global broadcast (255.255.255.255)
  static std::shared_ptr<UDPObservationSender> toAll(const uint port = 26078);

  bool open();
  void close();
  ~UDPObservationSender();
  void insertObservation(const RawObservation& data) final;

 private:
  UDPObservationSender(const std::string& target_ip, const uint port)
      : target_ip_(target_ip), port_(port) {}

  int fd_socket_{0};
  uint64_t sent_packets_{0};
  std::string target_ip_;
  uint16_t port_;

  struct addrinfo* target_addr_;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATION_SENDER_H_
