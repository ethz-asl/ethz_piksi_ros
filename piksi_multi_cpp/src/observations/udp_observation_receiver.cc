#include <piksi_multi_cpp/observations/udp_observation_receiver.h>
// udp
#include <arpa/inet.h>
#include <fcntl.h>  // File control definitions
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>  // UNIX standard function definitions

namespace piksi_multi_cpp {
UDPObservationReceiver::UDPObservationReceiver(
    ObservationsConsumer::Ptr consumer)
    : consumers_({consumer}) {}

void UDPObservationReceiver::addConsumer(
    piksi_multi_cpp::ObservationsConsumer::Ptr consumer) {
  std::lock_guard lock(consumers_lock_);  // obtain lock on consumers.
  consumers_.push_back(consumer);
  // lock is automatically released.
}

void UDPObservationReceiver::start(int port) {
  // set up socket
  struct addrinfo hints, *res;

  // set port
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;     // use IPv4 or IPv6, whichever
  hints.ai_socktype = SOCK_DGRAM;  // get full datagramm without IP header.
  hints.ai_flags = AI_PASSIVE;
  getaddrinfo(nullptr, std::to_string(port).c_str(), &hints, &res);

  // get socket & bind
  fd_socket_ = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (bind(fd_socket_, res->ai_addr, res->ai_addrlen) != 0) {
    close(fd_socket_);
  }

  // start thread.
  process_thread_ = std::thread(&UDPObservationReceiver::process, this);
}

void UDPObservationReceiver::stop() {
  stop_thread_ = true;
  shutdown(fd_socket_, SHUT_RD);  // trying to unblock the socket.
  if (process_thread_.joinable()) {
    process_thread_.join();
  }
  close(fd_socket_);
}

void UDPObservationReceiver::process() {
  while (!stop_thread_) {
    // read data from socket.
    socklen_t fromlen;
    struct sockaddr_storage addr;

    RawObservation buffer;
    buffer.resize(1600);  // larger than largest udp packets usually are.
    // TCP/IP usually has a MTU (maximum transmission unit) of about 1500

    if (recvfrom(fd_socket_, buffer.data(), buffer.size(), 0, (sockaddr*)&addr,
                 &fromlen)) {
      std::lock_guard lock(consumers_lock_);  // get lock on consumers

      // visit all consumers and insert data.
      for (auto consumer : consumers_) {
        consumer->insertObservation(buffer);
      }

      // lock is released once it gets out of scope here.
    }
  }
}

uint64_t UDPObservationReceiver::getPacketCount() { return received_packets_; }
}