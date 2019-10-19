
#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATIONS_RECEIVER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATIONS_RECEIVER_H_
#include <piksi_multi_cpp/observations/raw_observation_interface.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

namespace piksi_multi_cpp {

/*
 * Class that receives corrections via UDP and can write them to a class that
 * implements RawObservationsInterface
 *
 * Together with RawObservationsInterface, This implements a visitor pattern,
 * https://en.wikipedia.org/wiki/Visitor_pattern
 *
 *
 */
class UDPObservationReceiver {
 public:
  explicit UDPObservationReceiver(const RawObservationInterface::Ptr&);
  void start(int port);
  void stop();

  uint64_t getPacketCount();

  // Disable copy / assignement constructors.
  UDPObservationReceiver(const UDPObservationReceiver&) = delete;
  UDPObservationReceiver& operator=(const UDPObservationReceiver&) = delete;

 private:
  UDPObservationReceiver() = default;  // prevent default constructor

  void process();
  std::thread process_thread_;

  RawObservationInterface::Ptr consumer_;

  int fd_socket_{0};  // file descriptor for socket
  uint64_t received_packets_{0};
  std::atomic_bool stop_thread_{false};
};
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATIONS_RECEIVER_H_
