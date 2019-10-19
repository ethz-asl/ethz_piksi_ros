#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_FILE_OBSERVATION_LOGGER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_FILE_OBSERVATION_LOGGER_H_

#include <piksi_multi_cpp/observations/cb_to_raw_obs_converter.h>

namespace piksi_multi_cpp {

/*
 * Logger that writes Raw observations into file such that it can be used
 * for ppk.
 *
 * Consumes Raw Observations, thus has interface RawObservationInterface
 */
class FileObservationLogger : public RawObservationInterface {
 public:
  FileObservationLogger() {}
  bool open(const std::string& filename);
  void close();
  void insertObservation(const RawObservation& data) final;
  ~FileObservationLogger();

 private:
  FILE* log_file_{nullptr};  // not using fstream, but raw file for performance.

  // File pointer etc.
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_FILE_OBSERVATION_LOGGER_H_
