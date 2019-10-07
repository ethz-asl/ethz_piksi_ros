#ifndef PIKSI_MULTI_CPP_DEVICE_H_
#define PIKSI_MULTI_CPP_DEVICE_H_

#include <cstdint>

namespace piksi_multi_cpp {
class Device {
 public:
  Device() {}

  virtual bool open() = 0;
  virtual int32_t read(uint8_t* buff, uint32_t n, void* context) = 0;
  virtual void close() = 0;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_H_
