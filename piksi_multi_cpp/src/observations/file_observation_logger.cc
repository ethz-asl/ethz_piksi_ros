#include <piksi_multi_cpp/observations/file_observation_logger.h>
namespace piksi_multi_cpp {

bool FileObservationLogger::open(const std::string& filename) {
  if (log_file_ != nullptr) return false;  // can't open if another one is open.

  log_file_ = fopen(filename.c_str(), "wb");  // open binary file.
  return (log_file_ == nullptr);
}

void FileObservationLogger::close() {
  if (log_file_ != nullptr) {
    fclose(log_file_);
  }
}

void FileObservationLogger::insertObservation(
    const piksi_multi_cpp::RawObservation& data) {

  if (log_file_ != nullptr) {
    // write in chunks of 1byte for now. maybe blocked write might be a bit
    // faster. But because we don't write large amounts of data, it probably
    // doesn't matter too much
    fwrite(data.data(), sizeof(uint8_t), data.size(), log_file_);
  }
}

FileObservationLogger::~FileObservationLogger() { close(); }

}