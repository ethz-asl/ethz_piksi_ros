#include <iostream>
#include <set>
#include <rqt_gps_rtk_plugin/SBPDecoder.hpp>
#include <unistd.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

int main() {
  struct addrinfo hints, *res;
  int sockfd;

  socklen_t fromlen;
  struct sockaddr_storage addr;
  std::vector<uint8_t> buffer;
  buffer.resize(300);

// get host info, make socket, bind it to port 4950
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;  // use IPv4 or IPv6, whichever
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE;
  getaddrinfo(NULL, "26078", &hints, &res);
  sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  bind(sockfd, res->ai_addr, res->ai_addrlen);

  std::cout << "start" << std::endl;
  while (1) {
    size_t received_length;
    if ((received_length = recvfrom(sockfd, buffer.data(), buffer.size(), 0, (sockaddr *) &addr, &fromlen))) {
      SBP_MSG_OBS msg;

      if (SBPDecoder::decode<SBP_MSG_OBS>(buffer, &msg)) {
        std::cout << "msg found!" << std::endl;
        std::cout << "Part " << msg.header.n_obs.index + 1 << " of " << msg.header.n_obs.total_n << std::endl;
        std::cout << msg.header.tow << " " << (uint) (msg.header.n_obs.total_n) << std::endl;
        std::cout << msg.header.wn << std::endl;
        std::cout << msg.header.ns_residual << std::endl;
        std::cout << msg.obs.size() << std::endl;

        for (SBP_MSG_OBS_OBSERVATION obs : msg.obs) {
          std::cout << "\t";
          std::cout << ((float) obs.P) / 50.0 << " ";
          std::cout << ((float) obs.L_i) + ((float) obs.L_f) / 256.0 << " ";
          std::cout << ((float) obs.D_i) + ((float) obs.D_f) / 256.0 << " ";

          std::cout << ((float) obs.cn0) / 4.0 << " ";
          std::cout << (int) obs.lock << " ";
          std::cout << (int) obs.sid_sat << " ";
          std::cout << (int) obs.sid_code << " ";
          std::cout << obs.flags.pseodorange_valid << " ";
          std::cout << obs.flags.carrier_phase_valid << " ";
          std::cout << obs.flags.half_cycle_amb_resolv << " ";
          std::cout << obs.flags.doppler_valid << " ";
          std::cout << obs.flags.RAIM_excl << " " << std::endl;
        }

        std::cout << std::endl << std::flush;

      }
    }

  }

  return 0;
}