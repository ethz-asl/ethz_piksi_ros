#include <netdb.h>
#include <netinet/in.h>
#include <piksi_multi_cpp/observations/udp_observation_sender.h>
#include <ros/console.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>  // UNIX standard function definitions

namespace piksi_multi_cpp {

bool UDPObservationSender::open(uint port) {
  struct addrinfo hints, *res;

  // set port
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;     // use IPv4 or IPv6, whichever
  hints.ai_socktype = SOCK_DGRAM;  // get full datagramm without IP header.
  hints.ai_flags = AI_PASSIVE;
  getaddrinfo("255.255.255.255", std::to_string(port).c_str(), &hints, &res);

  // get socket & bind
  fd_socket_ = socket(res->ai_family, res->ai_socktype, res->ai_protocol);

  if (bind(fd_socket_, res->ai_addr, res->ai_addrlen) != 0) {
    ::close(fd_socket_);
    ROS_ERROR_STREAM(
        "Could not bind to UDP socket. Observations will not be received");
    return false;
  }
  return  true;
}

void UDPObservationSender::close() { ::close(fd_socket_); }

void UDPObservationSender::insertObservation(
    const piksi_multi_cpp::RawObservation& data) {
  // send udp package with SBP message.
  ::send(fd_socket_, data.data(), data.size(), 0);
}

UDPObservationSender::~UDPObservationSender() { close(); }
}  // namespace piksi_multi_cpp
