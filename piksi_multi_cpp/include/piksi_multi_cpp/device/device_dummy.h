#ifndef PIKSI_MULTI_CPP_DEVICE_DEVICE_DUMMY_H_
#define PIKSI_MULTI_CPP_DEVICE_DEVICE_DUMMY_H_

#include <unistd.h>
#include "piksi_multi_cpp/device/device.h"

namespace piksi_multi_cpp {

class DeviceDummy : public Device {
 public:
  inline DeviceDummy(const Identifier& id) : Device(id) {}

  inline bool openImpl() override { return true; }
  inline int32_t readImpl(uint8_t* buff, uint32_t n) const override {
    usleep(100);
    return 0;
  }
  inline int32_t writeImpl(std::vector<uint8_t> buff) const override {
    return 0;
  }
  inline void closeImpl() override {}
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_DEVICE_DUMMY_H_
