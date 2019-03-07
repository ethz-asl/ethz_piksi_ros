#include <iostream>
#include <set>
#include <rqt_gps_rtk_plugin/SBPStreamDecoder.hpp>
#include <rqt_gps_rtk_plugin/SBPDecoder.hpp>
#include <unistd.h>

void print(const SBP_MSG_OBS &msg) {
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
}

int main() {

  stdin = freopen(NULL, "rb", stdin);

  SBPStreamDecoder decoder(300, {SBP_MSG_TYPE::MSG_OBS});

  do {
    uint8_t temp;
    std::cin.read((char *) &temp, 1);

    if (decoder.addToBuffer(temp) == SBP_MSG_TYPE::MSG_OBS) {

      SBP_MSG_OBS msg;
      if (decoder.getCurrentMessage<SBP_MSG_OBS>(&msg)) {
        std::cout << "valid message!" << std::endl;
        print(msg);
      }

    }

    usleep(1000);
  } while (1);

  std::cout << "hello" <<
            std::endl;
  return 0;
}
