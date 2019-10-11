#include "piksi_multi_cpp/device/device_tcp.h"
#include <netinet/tcp.h>
#include <poll.h>
#include <ros/console.h>
#include <cstring>
#include <regex>

namespace piksi_multi_cpp {

bool DeviceTCP::parseId() {
  std::regex rgx("^tcp:\\/\\/(.+):(\\d{2,5})$");

  std::smatch match;
  if (std::regex_search(id_, match, rgx) && match.size() > 1) {
    host_ = match.str(1);
    std::stringstream(match.str(2)) >> tcp_port_;
    return true;
  }
  return false;
}

bool DeviceTCP::openSocket() {
  struct addrinfo connection_settings;

  // set up socket settings
  memset(&connection_settings, 0,
         sizeof(connection_settings));            // reset connection settings
  connection_settings.ai_family = AF_UNSPEC;      // allow ipv4/ipv6
  connection_settings.ai_socktype = SOCK_STREAM;  // TCP stream
  connection_settings.ai_protocol = IPPROTO_TCP;  // TCP

  // Set up endpoint address
  struct addrinfo* resolved;
  int result_addrinfo =
      getaddrinfo(host_.c_str(), std::to_string(tcp_port_).c_str(),
                  &connection_settings, &resolved);
  if (result_addrinfo != 0) {
    freeaddrinfo(resolved);
    ROS_ERROR_STREAM("Could not resolve address for device "
                     << id_ << ". Error:" << gai_strerror(result_addrinfo));
    return false;
  }

  // set up socket
  socket_fd_ =
      socket(resolved->ai_family, resolved->ai_socktype, resolved->ai_protocol);

  int synRetries = 2;  // Limit the amount of tcp connection tries, otherwise
                       // connect might block forever
  setsockopt(socket_fd_, IPPROTO_TCP, TCP_SYNCNT, &synRetries,
             sizeof(synRetries));

  // Try to connect
  int result_bind =
      connect(socket_fd_, resolved->ai_addr, resolved->ai_addrlen);

  if (result_bind != 0) {
    freeaddrinfo(resolved);
    ROS_ERROR_STREAM("Could not open socket for device "
                     << id_ << ". Error = " << errno << "/" << strerror(errno));
    return false;
  }

  // set receive timeout to 10s such that read methods blocks not forever when
  // something goes wrong
  struct timeval tv;
  tv.tv_sec = 10; /* 10 Secs Timeout */
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (struct timeval*)&tv,
             sizeof(struct timeval));

  return true;
}

bool DeviceTCP::open() {
  if (!parseId()) {
    return false;
  }

  if (!openSocket()) {
    return false;
  }

  // we have a valid connection!
  return true;
}

int32_t DeviceTCP::read(uint8_t* buff, uint32_t n) const {
  // todo: add wait until a package is received.

  ssize_t received_length =
      recvfrom(socket_fd_, (void*)buff, n, 0, nullptr, nullptr);

  if (received_length >= 0) {
    // all good or no data received
    return received_length;
  } else {
    ROS_WARN_STREAM("TCP error " << received_length << " while reading device "
                                 << id_);
    return 0;
  }
}

void DeviceTCP::close() { ::close(socket_fd_); }

}