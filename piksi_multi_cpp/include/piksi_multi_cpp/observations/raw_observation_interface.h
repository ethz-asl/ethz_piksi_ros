#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_RAW_OBSERVATION_INTERFACE_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_RAW_OBSERVATION_INTERFACE_H_

#include <memory>
#include <vector>

namespace piksi_multi_cpp {

/*
 * Defines an interface to <something> that can handle raw observation sbp
 * packages.
 *
 */

// Raw Observations is supposed to hold valid SBP messages.
typedef std::vector<uint8_t> RawObservation;

class RawObservationInterface {
 public:
  typedef std::shared_ptr<RawObservationInterface> Ptr;
  virtual void insertObservation(const RawObservation& data) = 0;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_RAW_OBSERVATION_INTERFACE_H_
