#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <piksi_multi_cpp/observations/udp_observation_sender.h>
#include <ros/console.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>  // UNIX standard function definitions
namespace piksi_multi_cpp {

std::shared_ptr<UDPObservationSender> UDPObservationSender::toHost(
    const std::string& host_ip, const uint port) {
  // can't use make_shared because constructor is private
  return std::shared_ptr<UDPObservationSender>(
      new UDPObservationSender(host_ip, port));
}

std::shared_ptr<UDPObservationSender> UDPObservationSender::toNetwork(
    const std::string& interface_name, const uint port) {
  // figure out network broadcast of that interface

  // create temporary socket
  struct ifreq ifr;
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  ifr.ifr_addr.sa_family = AF_INET;

  // set interface name
  std::string interface_name_delimited(interface_name);
  strncpy(
      ifr.ifr_name, interface_name_delimited.c_str(),
      std::min((unsigned long)(IFNAMSIZ), interface_name_delimited.size() + 1));

  // check if interface exists
  if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0) {
    ROS_ERROR_STREAM("Unable to get broadcast of interface " << interface_name);
    perror("SIOCGIFFLAGS");
    return nullptr;
  }

  // interface exists, check if it is running
  if ((ifr.ifr_flags & IFF_RUNNING) != IFF_RUNNING) {
    ROS_ERROR_STREAM("Unable to get broadcast of interface "
                     << interface_name << ", interface is down");
    return nullptr;
  }

  // Get brodcast address
  if (ioctl(fd, SIOCGIFBRDADDR, &ifr) == 0) {
    std::string network_broadcast =
        inet_ntoa(((struct sockaddr_in*)&ifr.ifr_broadaddr)->sin_addr);
    ROS_INFO_STREAM("BCAST ADDR of " << interface_name << " is "
                                     << network_broadcast);
    ::close(fd);
    // can't use make_shared because constructor is private
    return std::shared_ptr<UDPObservationSender>(
        new UDPObservationSender(network_broadcast, port));
  } else {
    ::close(fd);
    return nullptr;
  }
}

std::shared_ptr<UDPObservationSender> UDPObservationSender::toAll(
    const uint port) {
  // can't use make_shared because constructor is private
  return std::shared_ptr<UDPObservationSender>(
      new UDPObservationSender("255.255.255.255", port));
}

bool UDPObservationSender::open() {
  struct addrinfo hints;

  // setup address information
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;       // use IPv4 or IPv6, whichever
  hints.ai_socktype = SOCK_DGRAM;  // get full datagramm without IP header.
  hints.ai_flags = AI_PASSIVE;
  getaddrinfo(target_ip_.c_str(), std::to_string(port_).c_str(), &hints,
              &target_addr_);

  // set up socket
  fd_socket_ = socket(target_addr_->ai_family, target_addr_->ai_socktype,
                      target_addr_->ai_protocol);

  // test if socket is set up
  if (fd_socket_ < 0) {
    ROS_ERROR_STREAM("Could not set up UDP socket.");
    close();
    return false;
  }

  // enable broadcast possibilities for this socket.
  int broadcast = 1;
  if (setsockopt(fd_socket_, SOL_SOCKET, SO_BROADCAST, &broadcast,
                 sizeof(broadcast)) < 0) {
    ROS_ERROR_STREAM("Could not set up broadcast.");
    close();
    return false;
  }

  ROS_INFO_STREAM("UDPObservationSender set up with target = " << target_ip_
                                                               << ":" << port_);
  return true;
}  // namespace piksi_multi_cpp

void UDPObservationSender::close() {
  ::close(fd_socket_);
  freeaddrinfo(target_addr_);
}

void UDPObservationSender::insertObservation(
    const piksi_multi_cpp::RawObservation& data) {
  // send udp package with SBP message.
  int n = ::sendto(fd_socket_, data.data(), data.size(), 0,
                   target_addr_->ai_addr, target_addr_->ai_addrlen);
}

UDPObservationSender::~UDPObservationSender() { close(); }
}  // namespace piksi_multi_cpp
