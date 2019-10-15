#include <piksi_multi_cpp/observations/udp_observation_receiver.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <ros/console.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>  // UNIX standard function definitions

namespace piksi_multi_cpp {
UDPObservationReceiver::UDPObservationReceiver(
    const RawObservationInterface::Ptr& consumer)
    : consumer_(consumer) {}

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
    ROS_ERROR_STREAM(
        "Could not bind to UDP socket. Observations will not be received");
    close(fd_socket_);
    return;
  }

  // set socket to receive any broadcast/multicast
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr("255.255.255.255");
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);
  if (setsockopt(fd_socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq,
                 sizeof(mreq)) < 0) {
    ROS_ERROR_STREAM(
        "Could not set UDP listener to Multicast. Observations will not "
        "be received.");
    close(fd_socket_);
    return;
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
      // visit consumer and insert data.
      consumer_->insertObservation(buffer);

      // lock is released once it gets out of scope here.
    }
  }
}

uint64_t UDPObservationReceiver::getPacketCount() { return received_packets_; }
}