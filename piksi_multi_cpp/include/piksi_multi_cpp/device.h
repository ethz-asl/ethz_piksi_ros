#ifndef PIKSI_MULTI_CPP_DEVICE_H_
#define PIKSI_MULTI_CPP_DEVICE_H_

#include <cstdint>
#include <memory>
#include <set>
#include <string>

enum DeviceType { kUSB = 0 };

typedef std::string Identifier;
typedef std::set<Identifier> Identifiers;
inline bool identifierEqual(const Identifier& a, const Identifier& b) {
  return a.compare(b) == 0;
}

namespace piksi_multi_cpp {
class Device {
 public:
  virtual bool open() = 0;
  virtual int32_t read(uint8_t* buff, uint32_t n, void* context) = 0;
  virtual void close() = 0;

  // Factory method to create all devices.
  static std::shared_ptr<Device> create(DeviceType type, const Identifier& id);
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_H_
