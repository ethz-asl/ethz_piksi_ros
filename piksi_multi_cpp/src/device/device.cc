#include <piksi_multi_cpp/device/device_tcp.h>
#include <piksi_multi_cpp/device/device_usb.h>
#include <ros/console.h>
#include <memory>
#include <vector>

namespace piksi_multi_cpp {

Device::Device(const Identifier& id) : id_(id) {}

int32_t Device::read_redirect(uint8_t* buff, uint32_t n, void* context) {
  if (!context) {
    ROS_ERROR_STREAM("No context set.");
    return 0;
  }
  // Cast context to instance.
  Device* instance = static_cast<Device*>(context);
  // Execute instance's read function.
  return instance->read(buff, n);
}

int32_t Device::write_redirect(uint8_t* buff, uint32_t n, void* context) {
  if (!context) {
    ROS_ERROR_STREAM("No context set.");
    return 0;
  }
  // Cast context to instance.
  Device* instance = static_cast<Device*>(context);

  // Execute instance's write function.
  return instance->write(std::vector<uint8_t>(&buff[0], &buff[n]));
}

bool Device::open() {
  mtx_.lock();
  bool success = openImpl();
  mtx_.unlock();
  return success;
}

int32_t Device::read(uint8_t* buff, uint32_t n) {
  mtx_.lock();
  int32_t result = readImpl(buff, n);
  mtx_.unlock();
  return result;
}

int32_t Device::write(std::vector<uint8_t> buff) {
  mtx_.lock();
  int32_t result = writeImpl(buff);
  mtx_.unlock();
  return result;
}

void Device::close() {
  mtx_.lock();
  closeImpl();
  mtx_.unlock();
}

}  // namespace piksi_multi_cpp
