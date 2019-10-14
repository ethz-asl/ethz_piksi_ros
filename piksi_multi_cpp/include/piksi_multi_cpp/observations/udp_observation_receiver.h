
#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATIONS_RECEIVER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATIONS_RECEIVER_H_
#include <piksi_multi_cpp/observations/observations_consumer.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

namespace piksi_multi_cpp {

class UDPObservationReceiver {
 public:
  explicit UDPObservationReceiver(ObservationsConsumer::Ptr consumer);
  void addConsumer(ObservationsConsumer::Ptr consumer);
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
  std::mutex consumers_lock_;
  std::vector<ObservationsConsumer::Ptr> consumers_;


  int fd_socket_{0};  // file descriptor for socket
  uint64_t received_packets_{0};
  std::atomic_bool stop_thread_{false};
};
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_UDP_OBSERVATIONS_RECEIVER_H_
