#include <iostream>
#include <set>
#include <rqt_gps_rtk_plugin/SBPDecoder.hpp>
#include <unistd.h>


int main() {

  enum class ReceiverState : uint8_t {
    Ready = 0,
    Synced = 1,
    HeaderFound = 2,
    Complete = 3
  };

  std::set<SBP_MSG_TYPE> valid_types;
  valid_types.insert(SBP_MSG_TYPE::MSG_OBS);

  stdin = freopen(NULL, "rb", stdin);
  std::vector<uint8_t> buffer;
  buffer.resize(300);

  ReceiverState state = ReceiverState::Ready;

  size_t buffer_ptr = 0;
  SBP_MSG_HEADER header;
  size_t total_length = 0;

  do {

    if (state == ReceiverState::Ready || buffer_ptr >= buffer.size() - 1) {
      buffer_ptr = 0;
      //clear buffer
      for (size_t i = 0; i < buffer.size(); i++) {
        buffer[i] = 0x00;
      }

    }
    std::cin.read((char *) &buffer[buffer_ptr], 1);


    // READY => SYNC
    if (state == ReceiverState::Ready
        && SBPDecoder::isSync(buffer[buffer_ptr])) {
      std::cout << "." << std::endl;
      state = ReceiverState::Synced;
    } else if (state == ReceiverState::Synced && buffer_ptr >= sizeof(SBP_MSG_HEADER)) {

      if (SBPDecoder::validHeader(buffer, &header, valid_types)) {
        state = ReceiverState::HeaderFound;
        total_length = header.length + sizeof(SBP_MSG_HEADER) + 2;
      } else {
        state = ReceiverState::Ready;
      }
      // already read more bytes than header, but not valid header => back to ready.
      // SYNC => READY


    } else if (state == ReceiverState::HeaderFound) {

      // read rest of message
      if (buffer_ptr >= total_length) {
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

          //clear buffer
          for (size_t i = 0; i < buffer.size(); i++) {
            buffer[i] = 0x00;
          }

        }
        state = ReceiverState::Ready;
      }

    }

    buffer_ptr++;

    usleep(1000);
  } while (1 );

  std::cout << "hello" << std::endl;
  return 0;
}