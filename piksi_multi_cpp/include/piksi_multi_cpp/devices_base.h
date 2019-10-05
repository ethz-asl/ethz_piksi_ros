#ifndef PIKSI_MULTI_CPP_DEVICES_BASE_H_
#define PIKSI_MULTI_CPP_DEVICES_BASE_H_

namespace piksi_multi_cpp {
class DevicesBase {
 public:
  DevicesBase() {}

  virtual bool open() = 0;
  virtual bool read() = 0;
  virtual bool close() = 0;
};
}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICES_BASE_H_
