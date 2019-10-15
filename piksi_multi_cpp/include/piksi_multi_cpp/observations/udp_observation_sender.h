
#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATION_SENDER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATION_SENDER_H_
#include <piksi_multi_cpp/observations/cb_to_raw_obs_converter.h>
namespace piksi_multi_cpp {
class UDPObservationSender : public RawObservationInterface {
 public:
  UDPObservationSender() {}

  bool open(uint port);
  void close();

  ~UDPObservationSender();
  void insertObservation(const RawObservation& data) final;

 private:
  int fd_socket_{0};
  uint64_t sent_packets_{0};
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATION_SENDER_H_
