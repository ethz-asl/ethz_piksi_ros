#include "piksi_multi_cpp/device/device_serial.h"

#include <libserialport.h>
#include <ros/console.h>
#include <regex>

namespace piksi_multi_cpp {

DeviceSerial::DeviceSerial(const Identifier& id) : Device(id), port_(nullptr) {}

bool DeviceSerial::parseId() {
  std::regex rgx("^(.+)@(\\d{4,7})$");

  std::smatch match;
  if (std::regex_search(id_, match, rgx) && match.size() > 1) {
    portname_ = match.str(1);
    std::stringstream(match.str(2)) >> baudrate_;
    return true;
  }
  return false;
}

bool DeviceSerial::allocatePort() {
  sp_return result = sp_get_port_by_name(portname_.c_str(), &port_);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("No serial port named " << portname_);
    return false;
  }

  return true;
}

bool DeviceSerial::setBaudRate() {
  sp_return result = sp_set_baudrate(port_, baudrate_);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot set baud rate: " << result);
    return false;
  }
  ROS_DEBUG("Configured the baud rate.");
  return true;
}

bool DeviceSerial::setPortDefaults() {
  // Configuration.
  // https://github.com/swift-nav/libsbp/blob/master/c/example/example.c
  sp_return result = sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot set flow control: " << result);
    closeImpl();
    return false;
  }
  ROS_DEBUG("Configured the flow control.");

  result = sp_set_bits(port_, 8);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot set data bits: " << result);
    closeImpl();
    return false;
  }
  ROS_DEBUG("Configured the number of data bits.");

  result = sp_set_parity(port_, SP_PARITY_NONE);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot set parity: " << result);
    closeImpl();
    return false;
  }
  ROS_DEBUG("Configured the parity.");

  result = sp_set_stopbits(port_, 1);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot set stop bits: " << result);
    closeImpl();
    return false;
  }
  ROS_DEBUG("Configured the number of stop bits.");
  return true;
}

bool DeviceSerial::openImpl() {
  if (!parseId()) {
    return false;
  }

  // Allocate port.
  if (!allocatePort()) {
    closeImpl();
    return false;
  }

  // Open port.
  sp_return result = sp_open(port_, SP_MODE_READ_WRITE);
  if (result != SP_OK) {
    ROS_ERROR("Cannot open port %s: %d", sp_get_port_name(port_), result);
    ROS_ERROR_COND(result == SP_ERR_FAIL, "%s", sp_last_error_message());
    closeImpl();
    return false;
  }
  ROS_INFO_STREAM("Opened port: " << sp_get_port_name(port_));

  // set Baudrate
  if (!setPortDefaults()) {
    closeImpl();
    ROS_ERROR_STREAM("Cannot set port defaults on port "
                     << sp_get_port_name(port_));
    return false;
  }

  if (!setBaudRate()) {
    closeImpl();
    ROS_ERROR_STREAM("Cannot set baudrate on port " << sp_get_port_name(port_));
    return false;
  }

  return true;
}

int32_t DeviceSerial::writeImpl(std::vector<uint8_t> buff) const {
  if (!port_) {
    ROS_ERROR("Port not opened.");
  }
  // Use blocking write.
  const int kTimeoutMs = 1000;
  int result = sp_blocking_write(port_, buff.data(), buff.size(), kTimeoutMs);
  ROS_ERROR_COND(result < 0, "Serial write failed.");
  ROS_ERROR_COND(result == SP_ERR_FAIL, "%s", sp_last_error_message());
  return result;
}

int32_t DeviceSerial::readImpl(uint8_t* buff, uint32_t n) const {
  if (!port_) {
    ROS_ERROR_STREAM("Port not opened.");
    return 0;
  }

  auto received = sp_blocking_read(port_, buff, n, 10000);

  if (received >= 0 && static_cast<uint32_t>(received) < n) {
    ROS_ERROR_STREAM("Serial read timeout received bytes:"
                     << received << " requested bytes: " << n
                     << " device: " << id_);
    received = SP_ERR_FAIL;
  }

  return received;
}

void DeviceSerial::closeImpl() {
  if (port_) {
    sp_return result = sp_close(port_);
    if (result != SP_OK) {
      ROS_ERROR("Cannot close %s properly.", sp_get_port_name(port_));
    } else {
      ROS_INFO("Closing port %s", sp_get_port_name(port_));
    }
    sp_free_port(port_);
    port_ = nullptr;
  }
}

}  // namespace piksi_multi_cpp
