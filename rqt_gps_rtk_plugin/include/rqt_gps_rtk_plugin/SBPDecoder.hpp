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
  uint RAIM_excl:1;
  uint reserved:3;
  uint doppler_valid:1;
  uint half_cycle_amb_resolv:1;
  uint carrier_phase_valid:1;
  uint pseodorange_valid:1;
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

  // default for static length messages
  template<typename T>
  static bool decode(const std::vector<uint8_t> &buffer, T *message);

  // helper method for synching
  static bool isSync(const uint8_t value);

  // Check partial message (to get length for buffer)
  static bool validHeader(const std::vector<uint8_t> &buffer,
                          SBP_MSG_HEADER *header,
                          const std::set<SBP_MSG_TYPE> &valid_types = {});

 private:

  // check full message
  static bool checkMessage(const std::vector<uint8_t> &buffer, SBP_MSG_HEADER *header);
  // checksum IRCC
  static uint16_t calculateChecksum(const std::vector<uint8_t> &buffer, const uint16_t offset, const uint16_t length);

/* CRC16 implementation acording to CCITT standards */
  static const uint16_t crc16tab[256];

};

#endif //CORRECTIONDECODER_H
