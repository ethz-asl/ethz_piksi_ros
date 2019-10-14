#ifndef PIKSI_MULTI_CPP_OBSERVATIONS_FILE_OBSERVATION_LOGGER_H_
#define PIKSI_MULTI_CPP_OBSERVATIONS_FILE_OBSERVATION_LOGGER_H_
#include <piksi_multi_cpp/observations/raw_observation_handler.h>
namespace piksi_multi_cpp {
class FileObservationLogger : public RawObservationHandler {
 public:
  FileObservationLogger() : RawObservationHandler() {}

  void write(std::vector<uint8_t> data);

 private:
  // File pointer etc.
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_OBSERVATIONS_FILE_OBSERVATION_LOGGER_H_
