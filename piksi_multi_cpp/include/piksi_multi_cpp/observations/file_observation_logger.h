#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_FILE_OBSERVATION_LOGGER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_FILE_OBSERVATION_LOGGER_H_

#include <piksi_multi_cpp/observations/cb_to_raw_obs_converter.h>
#include <mutex>

namespace piksi_multi_cpp {

/*
 * Logger that writes raw observations into file such that it can be used
 * for PPK.
 * Logger consumes raw observations.
 */
class FileObservationLogger : public RawObservationInterface {
 public:
  FileObservationLogger() {}
  /**
   * Creates and opens log file. In case file name already exists a number is
   * appended to filename so that old files are not overwritten.
   */
  bool open(const std::string& filename);

  /**
   * Close log file
   */
  void close();

  /**
   * Return status whether logger is currently running
   */
  bool isLogging();

  /**
   * Write observations to binary file
   */
  void insertObservation(const RawObservation& data) final;

  /**
   * Destructor making sure log file is closed.
   */
  ~FileObservationLogger();

 private:
  // File pointer.
  FILE* log_file_{nullptr};  // not using fstream, but raw file for performance.

  // mutex to lock file when writing
  std::mutex file_mtx_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_FILE_OBSERVATION_LOGGER_H_
