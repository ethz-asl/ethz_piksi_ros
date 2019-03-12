#ifndef CORRECTIONDECODER_H
#define CORRECTIONDECODER_H

#include <iostream>
#include <vector>
#include <set>
#include <cstring>
#include <typeindex>
#include <map>
#include <sstream>
#include <unordered_map>

#include <typeinfo>

// only contains currently parseable messages
enum class SBP_MSG_TYPE : uint16_t {
  INVALID = 0xFFFF,
  MSG_OBS = 0x004A,
  MSG_BASELINE_ECEF = 0x0202
};


// note: packed attribute mandatory for low-level parsing.
//       Otherwise the compiler might align variables with padding etc.
//
// Definition source: SBP Manual v2.4.7
// Available here: https://support.swiftnav.com/customer/en/portal/articles/2492810-swift-binary-protocol

// Header as defined in chapter "2 Message Framing Structure".
typedef struct __attribute__ ((packed)) {
  uint8_t preamble;
  SBP_MSG_TYPE message_type;
  uint16_t sender;
  uint8_t length;
} SBP_MSG_HEADER;

// Struct for measurement status flags as defined in field
// definition 6.6.1 / Chapter 6.6
typedef struct __attribute__ ((packed)) {
  uint pseodorange_valid:1;
  uint carrier_phase_valid:1;
  uint half_cycle_amb_resolv:1;
  uint doppler_valid:1;
  uint reserved:3;
  uint RAIM_excl:1;
} SBP_MSG_OBS_OBSERVATION_FLAGS;

// Struct for number of observations as defined in Chapter 6.6
// (field n_obs in table)
typedef struct __attribute__ ((packed)) {
  uint index:4;
  uint total_n:4;     // upper and lower nibble (:4)
} SBP_MSG_OBS_OBSERVATION_NOBS;

// Struct for observation header as defined in Chapter 6.6 (Table 6.6.1).
typedef struct __attribute__ ((packed)) {
  uint32_t tow;
  int32_t ns_residual;
  uint16_t wn;
  SBP_MSG_OBS_OBSERVATION_NOBS n_obs;
} SBP_MSG_OBS_HEADER;

// Struct for observation header as defined in Chapter 6.6 (Table 6.6.1)
typedef struct __attribute__ ((packed)) {
  uint32_t P;   // pseudoragne
  int32_t L_i;  // carrier phase integer cycles
  uint8_t L_f;  // carrier phase fractional part
  int16_t D_i;  // Doppler whole Hz
  uint8_t D_f;  // Doppler fractional part
  uint8_t cn0;  // Carrier-to noise density
  uint8_t lock; // lock timer
  SBP_MSG_OBS_OBSERVATION_FLAGS flags;
  uint8_t sid_sat; // sat id
  uint8_t sid_code; // satellite constellation + code

} SBP_MSG_OBS_OBSERVATION;

typedef struct __attribute__((packed)) {
  uint32_t tow;
  int32_t x;
  int32_t y;
  int32_t z;
  uint16_t accuracy;
  uint8_t nsats;
  uint8_t flags;
} SBP_MSG_BASELINE_ECEF;

// non packed message, as it is not used for deserialization.
class SBP_MSG_OBS {
 public:
  SBP_MSG_OBS_HEADER header;
  std::vector<SBP_MSG_OBS_OBSERVATION> observations;
  
  std::string str() {
    std::map<uint8_t, std::string> code_map = {
        {0, "GPS L1CA"},
        {1, "GPS L2CM"},
        {2, "SBAS L1CA"},
        {3, "GLO L1CA"},
        {4, "GLO L2CA"},
        {5, "GPS L1P"},
        {6, "GPS L2P"},
        {12, "BDS2 B1"},
        {13, "BDS2 B2"},
        {14, "GAL E1B"},
        {20, "GAL E7I"}
    };

    std::stringstream sstream;
    sstream << "Correction Part " << header.n_obs.index + 1 << " of "
            << header.n_obs.total_n << std::endl;
    sstream << "GPS Week:\t\t " << header.wn << std::endl;
    sstream << "Time of week:\t\t" << header.tow << " [ms]" << std::endl;
    sstream << "Observations:\t\t" << observations.size() << std::endl;
    sstream << "------------------------------------------\t\t" << std::endl;

    for (const SBP_MSG_OBS_OBSERVATION& obs : observations) {
      if (code_map.count(obs.sid_code)) {
        sstream << code_map[obs.sid_code] << " / " << (int) obs.sid_sat << "\t";
      } else {
        sstream << "???? / " << (int) obs.sid_sat << "\t";
      }

      //add carrier noise density
      sstream << ((float) obs.cn0) / 4.0 << " dB Hz\t\t";

      if (obs.flags.RAIM_excl) {
        sstream << "!EXCLUDED! ";
      } else {
        sstream << "           ";
      }

      if (obs.flags.pseodorange_valid) {
        sstream << "PSDO ";
      } else {
        sstream << "     ";
      }

      if (obs.flags.carrier_phase_valid) {
        sstream << "CARR ";
      } else {
        sstream << "     ";
      }

      if (obs.flags.half_cycle_amb_resolv) {
        sstream << "CYCL ";
      } else {
        sstream << "     ";
      }

      if (obs.flags.doppler_valid) {
        sstream << "DPLR ";
      } else {
        sstream << "     ";
      }

      sstream << std::endl;
    }
    return sstream.str();
  }

};

class SBPDecoder {

 public:

  static const std::unordered_map<std::type_index, SBP_MSG_TYPE> MessageTypes;

  // default for static length messages -
  // templated implementation in header file (important)
  template<class T>
  static bool decode(const std::vector<uint8_t>& buffer, T* message) {
    std::cout << "DECODE A" << std::endl;
    SBP_MSG_HEADER header;
    if (!checkMessage(buffer, &header)) {
      return false;
    }

    if (header.message_type
        != SBPDecoder::MessageTypes.at(std::type_index(typeid(T)))) {
      return false;
    }

    if (buffer.size() < (sizeof(SBP_MSG_HEADER) + sizeof(T) + 2)) {
      return false;
    }

    memcpy(message, buffer.data() + sizeof(SBP_MSG_HEADER), sizeof(T));
    return true;
  }

  // helper method for synching
  static bool isSync(const uint8_t value);

  // Check partial message (to get length for buffer)
  static bool validHeader(const std::vector<uint8_t>& buffer,
                          SBP_MSG_HEADER* header,
                          const std::set<SBP_MSG_TYPE>& valid_types = {});

  static size_t getMessageSize(const SBP_MSG_HEADER& header);

  static bool checkMessage(const std::vector<uint8_t>& buffer,
                           SBP_MSG_HEADER* header = nullptr);
 private:

  // check full message

  // checksum IRCC
  static uint16_t calculateChecksum(const std::vector<uint8_t>& buffer,
                                    const uint16_t offset,
                                    const uint16_t length);

/* CRC16 implementation acording to CCITT standards */
  static const uint16_t crc16tab[256];
};

// has to be outside of class (important)
// override specialization for variable length observations
// template attribute necessary, otherwise not considered overwrite
template<>
inline bool SBPDecoder::decode(const std::vector<uint8_t>& buffer,
                               SBP_MSG_OBS* message) {
  SBP_MSG_HEADER header;
  if (!checkMessage(buffer, &header)) {
    return false;
  }

  if (header.message_type != SBP_MSG_TYPE::MSG_OBS) {
    return false;
  }

  //get Observation header
  memcpy(&message->header,
         buffer.data() + sizeof(SBP_MSG_HEADER),
         sizeof(SBP_MSG_OBS_HEADER));

  // we use the calculated number of observations, not n_obs.
  size_t n_obs_calc = (header.length - 11) / 17;
  message->observations.resize(n_obs_calc);

  for (size_t i = 0; i < n_obs_calc; i++) {
    const uint8_t* position = buffer.data() + sizeof(SBP_MSG_HEADER) +
        sizeof(SBP_MSG_OBS_HEADER) + i * sizeof(SBP_MSG_OBS_OBSERVATION);

    memcpy(&message->observations[i],
           position,
           sizeof(SBP_MSG_OBS_OBSERVATION));
  }
  return true;
}

#endif //CORRECTIONDECODER_H
