#include <piksi_multi_cpp/observations/file_observation_logger.h>
namespace piksi_multi_cpp {

bool FileObservationLogger::open(const std::string& filename) {
  if (log_file_ != nullptr) return false;  // can't open if another one is open.

  int file_number = 0;
  std::string unique_filename = filename + ".sbp";
  while ((log_file_ = fopen(unique_filename.c_str(), "r"))) {
    file_number++;
    unique_filename = filename + "_" + std::to_string(file_number) + ".sbp";
    fclose(log_file_);
  }

  log_file_ = fopen(unique_filename.c_str(), "wb");  // open binary file.
  return (log_file_ == nullptr);
}

void FileObservationLogger::close() {
  if (log_file_ != nullptr) {
    fclose(log_file_);
    log_file_ = nullptr;
  }
}

bool FileObservationLogger::isLogging() { return (log_file_ != nullptr); }

void FileObservationLogger::insertObservation(
    const piksi_multi_cpp::RawObservation& data) {
  if (log_file_ != nullptr) {
    file_mtx_.lock();
    // write in chunks of 1byte for now. maybe blocked write might be a bit
    // faster. But because we don't write large amounts of data, it probably
    // doesn't matter too much
    fwrite(data.data(), sizeof(uint8_t), data.size(), log_file_);
    file_mtx_.unlock();
  }
}

FileObservationLogger::~FileObservationLogger() { close(); }

}  // namespace piksi_multi_cpp
