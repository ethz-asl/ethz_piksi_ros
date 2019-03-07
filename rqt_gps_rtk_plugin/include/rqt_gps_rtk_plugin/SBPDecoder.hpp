#ifndef CORRECTIONDECODER_H
#define CORRECTIONDECODER_H

#include <iostream>
#include <vector>
#include <set>
#include <cstring>
#include <typeindex>
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

typedef struct __attribute__ ((packed)) {
  uint8_t preamble;
  SBP_MSG_TYPE message_type;
  uint16_t sender;
  uint8_t length;
} SBP_MSG_HEADER;

// struct for measurement status flags
typedef struct __attribute__ ((packed)) {
  uint pseodorange_valid:1;
  uint carrier_phase_valid:1;
  uint half_cycle_amb_resolv:1;
  uint doppler_valid:1;
  uint reserved:3;
  uint RAIM_excl:1;
} SBP_MSG_OBS_OBSERVATION_FLAGS;

// struct for n_obs
typedef struct __attribute__ ((packed)) {
  uint index:4;
  uint total_n:4;     // upper and lower nibble (:4)
} SBP_MSG_OBS_OBSERVATION_NOBS;

typedef struct __attribute__ ((packed)) {
  uint32_t tow;
  int32_t ns_residual;
  uint16_t wn;
  SBP_MSG_OBS_OBSERVATION_NOBS n_obs;
} SBP_MSG_OBS_HEADER;

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
typedef struct {
  SBP_MSG_OBS_HEADER header;
  std::vector<SBP_MSG_OBS_OBSERVATION> obs;
} SBP_MSG_OBS;

class SBPDecoder {

 public:

  static const std::unordered_map<std::type_index, SBP_MSG_TYPE> MessageTypes;

  // default for static length messages - templated implementation in header file (important)
  template<class T>
  static bool decode(const std::vector<uint8_t> &buffer, T *message) {
    std::cout << "DECODE A" << std::endl;
    SBP_MSG_HEADER header;
    if (!checkMessage(buffer, &header)) {
      return false;
    }

    if (header.message_type != SBPDecoder::MessageTypes.at(std::type_index(typeid(T)))) {
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
  static bool validHeader(const std::vector<uint8_t> &buffer,
                          SBP_MSG_HEADER *header,
                          const std::set<SBP_MSG_TYPE> &valid_types = {});

  static size_t getMessageSize(const SBP_MSG_HEADER &header);

  static bool checkMessage(const std::vector<uint8_t> &buffer, SBP_MSG_HEADER *header = nullptr);
 private:

  // check full message

  // checksum IRCC
  static uint16_t calculateChecksum(const std::vector<uint8_t> &buffer, const uint16_t offset, const uint16_t length);

/* CRC16 implementation acording to CCITT standards */
  static const uint16_t crc16tab[256];
};


// has to be outside of class (important)
// override specialization for variable length observations
// template attribute necessary, otherwise not considered overwrite
// (also nicer alternative to regular template specialization in this case)
template<>
inline bool SBPDecoder::decode(const std::vector<uint8_t> &buffer, SBP_MSG_OBS *message) {
  SBP_MSG_HEADER header;
  if (!checkMessage(buffer, &header)) {
    return false;
  }

  if (header.message_type != SBP_MSG_TYPE::MSG_OBS) {
    return false;
  }

  //get Observation header
  memcpy(&message->header, buffer.data() + sizeof(SBP_MSG_HEADER), sizeof(SBP_MSG_OBS_HEADER));

  // we use the calculated number of observations, not n_obs. N_OBS can be split amongst multiple messages and
  // contains indexing info (see datasheet).
  size_t n_obs_calc = (header.length - 11) / 17;
  message->obs.resize(n_obs_calc);

  for (size_t i = 0; i < n_obs_calc; i++) {
    const uint8_t *position = buffer.data() + sizeof(SBP_MSG_HEADER) +
        sizeof(SBP_MSG_OBS_HEADER) + i * sizeof(SBP_MSG_OBS_OBSERVATION);

    memcpy(&message->obs[i], position, sizeof(SBP_MSG_OBS_OBSERVATION));
  }
  return true;
}

#endif //CORRECTIONDECODER_H
