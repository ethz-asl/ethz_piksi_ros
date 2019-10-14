#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_OBSERVATIONS_CONSUMER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_OBSERVATIONS_CONSUMER_H_
#include <libsbp/observation.h>
#include <libsbp/system.h>
#include <vector>
#include <memory>

namespace piksi_multi_cpp {

/*
 * Defines an interface to <something> that can handle raw observation sbp
 * packages.
 *
 */

// Raw Observations is supposed to hold valid SBP messages.
typedef std::vector<uint8_t> RawObservation;

class ObservationsConsumer {
 public:
  typedef std::shared_ptr<ObservationsConsumer> Ptr;
  virtual void insertObservation(RawObservation& data) = 0;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_OBSERVATIONS_CONSUMER_H_
