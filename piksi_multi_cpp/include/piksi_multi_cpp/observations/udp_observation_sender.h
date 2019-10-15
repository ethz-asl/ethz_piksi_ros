
#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATION_SENDER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATION_SENDER_H_
#include <piksi_multi_cpp/observations/raw_observation_handler.h>
namespace piksi_multi_cpp {
class UDPObservationSender : public RawObservationInterface {
 public:
  UDPObservationSender() {}

 protected:
  void insertObservation(const RawObservation& data) final;

 private:
  int fd_socket_{0};
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATION_SENDER_H_
