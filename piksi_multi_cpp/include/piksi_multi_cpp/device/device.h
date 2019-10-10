#ifndef PIKSI_MULTI_CPP_DEVICE_DEVICE_H_
#define PIKSI_MULTI_CPP_DEVICE_DEVICE_H_

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace piksi_multi_cpp {

typedef std::string SerialNumber;
typedef std::set<SerialNumber> SerialNumbers;
inline bool equalSerialNumber(const SerialNumber& a, const SerialNumber& b) {
  return a.compare(b) == 0;
}

class Device {
 public:
  enum DeviceType { kUSB = 0 };

  Device(const SerialNumber& sn);
  virtual bool open() = 0;
  virtual int32_t read(uint8_t* buff, uint32_t n) const = 0;
  virtual void close() = 0;
  inline std::string getID() const { return serial_number_; }

  // This function will be passed to sbp_process.
  // libsbp is a C interface and does not allow binding a non-static member
  // function using std::bind. Thus we bind this static member function that
  // points on the context's read function.
  // WARNING: Requires sbp_state_set_io_context to be called first.
  static int32_t read_redirect(uint8_t* buff, uint32_t n, void* context);

 protected:
  SerialNumber serial_number_;
};

typedef std::shared_ptr<Device> DevicePtr;
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_DEVICE_H_
