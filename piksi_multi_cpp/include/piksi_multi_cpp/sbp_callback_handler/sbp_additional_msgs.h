#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_ADDITIONAL_MSGS_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_ADDITIONAL_MSGS_H_
#include <libsbp/sbp.h>
#include <libsbp/observation.h>
#include <cstring>
#include <vector>

namespace piksi_multi_cpp {
/* Specialization for observation with variable length... */
typedef struct {
  observation_header_t header;
  std::vector<packed_obs_content_t> obs;

  inline void fromBuffer(uint8_t* buff, uint8_t len) {
    memcpy(&header, buff, sizeof(observation_header_t));

    uint8_t num_obs =
        (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);
    obs.resize(num_obs);
    memcpy(obs.data(), buff + sizeof(observation_header_t),
           num_obs * sizeof(packed_obs_content_t));
  }

  inline void toBuffer(uint8_t* buff) {
    // we just assume buffer is large enough. dangerous.
    memcpy(buff, &header, sizeof(observation_header_t));
    memcpy(buff + sizeof(observation_header_t), obs.data(),
           obs.size() * sizeof(packed_obs_content_t));
  }

  inline uint8_t length() {
    return sizeof(observation_header_t) +
           sizeof(packed_obs_content_t) * obs.size();
  }
} msg_obs_t_var;
}  // namespace piksi_multi_cpp
#endif  // PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_ADDITIONAL_MSGS_H_
