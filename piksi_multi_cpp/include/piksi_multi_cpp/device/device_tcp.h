
#ifndef PIKSI_MULTI_CPP_DEVICE_DEVICE_TCP_H_
#define PIKSI_MULTI_CPP_DEVICE_DEVICE_TCP_H_

#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <boost/format.hpp>
#include "piksi_multi_cpp/device/device.h"

namespace piksi_multi_cpp {

class DeviceTCP : public Device {
 public:
  /*
   * Identifier is expected to be of type "<ip>:<port>"
   */
  DeviceTCP(const Identifier& id) : Device(id) {
    // we don't do anything else here, leave the parsing to the open method.
  }

  bool openImpl() override;
  bool openSocket();
  bool parseId();
  int32_t readImpl(uint8_t* buff, uint32_t n) const override;
  int32_t writeImpl(std::vector<uint8_t> buff) const override;
  void closeImpl() override;

 private:
  int socket_fd_;     // socket file descriptor
  uint tcp_port_;     // TCP port.
  std::string host_;  // hostname or IP as string
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_DEVICE_TCP_H_
