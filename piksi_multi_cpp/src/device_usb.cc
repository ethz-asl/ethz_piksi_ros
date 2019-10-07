#include "piksi_multi_cpp/device_usb.h"

#include <libserialport.h>
#include <ros/console.h>

const int kInterfaceNumber = 1;

namespace piksi_multi_cpp {

DeviceUSB::DeviceUSB(const USBIdentifier& id) : port_(nullptr), id_(id) {}

USBIdentifiers DeviceUSB::getAllIdentifiers() {
  USBIdentifiers identifiers;

  // Get list of all ports.
  struct sp_port** ports;
  sp_return result = sp_list_ports(&ports);

  if (result != SP_OK) {
    ROS_WARN_STREAM("No USB devices detected: " << result);
    return identifiers;
  }

  // Identify Piksi Multi among ports.
  for (int i = 0; ports[i]; i++) {
    // Check if port belongs to a Piksi Multi.
    USBIdentifier id = identifyPiksi(ports[i]);
    // Add serial to identifier set.
    if (id) {
      identifiers.insert(id);
    }
  }

  ROS_INFO("%lu Piksi Multi identified on USB.", identifiers.size());
  sp_free_port_list(ports);

  return identifiers;
}

void DeviceUSB::printPorts() {
  struct sp_port** ports;
  sp_return result = sp_list_ports(&ports);
  if (result == SP_OK) {
    for (int i = 0; ports[i]; i++) {
      printDeviceInfo(ports[i]);
    }
    sp_free_port_list(ports);
  } else {
    ROS_ERROR_STREAM("No serial devices detected: " << result);
  }
}

bool DeviceUSB::allocatePort(const USBIdentifier& id) {
  // Find any serial port with id.
  struct sp_port** ports;
  sp_return result = sp_list_ports(&ports);
  if (result < 0) {
    ROS_ERROR_STREAM("No serial devices detected: " << result);
    return false;
  }

  for (int i = 0; ports[i]; i++) {
    USBIdentifier id = identifyPiksi(ports[i]);
    if (strcmp(id, id) == 0) {
      result = sp_copy_port(ports[i], &port_);
      if (result == SP_OK) {
        break;
      } else {
        ROS_ERROR_STREAM("Cannot copy port: " << result);
      }
    }
  }

  sp_free_port_list(ports);

  if (!port_) {
    ROS_ERROR("Failed to allocate port.");
    return false;
  } else {
    ROS_INFO_STREAM("Allocated port for ID: " << id_);
    return true;
  }
}

bool DeviceUSB::open() {
  // Allocate port.
  if (!allocatePort(id_)) {
    close();
    return false;
  }

  // Open port.
  ROS_INFO_STREAM("Opening port: " << sp_get_port_name(port_));
  sp_return result = sp_open(port_, SP_MODE_READ);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot open port: " << result);
    close();
    return false;
  }

  // Configuration.
  // https://github.com/swift-nav/libsbp/blob/master/c/example/example.c
  result = sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot set flow control: " << result);
    close();
    return false;
  }
  ROS_INFO("Configured the flow control.");

  result = sp_set_bits(port_, 8);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot set data bits: " << result);
    close();
    return false;
  }
  ROS_INFO("Configured the number of data bits.");

  result = sp_set_parity(port_, SP_PARITY_NONE);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot set parity: " << result);
    close();
    return false;
  }
  ROS_INFO("Configured the parity.");

  result = sp_set_stopbits(port_, 1);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot set stop bits: " << result);
    close();
    return false;
  }
  ROS_INFO("Configured the number of stop bits.");

  return false;
}

int32_t DeviceUSB::read(uint8_t* buff, uint32_t n, void* context) {
  if (!port_) {
    ROS_ERROR_STREAM("Port not opened.");
    return 0;
  }

  (void)context;
  return sp_blocking_read(port_, buff, n, 0);
}

void DeviceUSB::close() {
  if (port_) {
    sp_return result = sp_close(port_);
    if (result != SP_OK) {
      ROS_ERROR("Cannot close %s properly.", sp_get_port_name(port_));
    }
    sp_free_port(port_);
    port_ = nullptr;
  }
}

USBIdentifier DeviceUSB::identifyPiksi(struct sp_port* port) {
  if (!port) return nullptr;

  // Get vendor and product ID to identify Piksi Multi.
  int usb_vid = -1;
  int usb_pid = -1;
  sp_return result = sp_get_port_usb_vid_pid(port, &usb_vid, &usb_pid);
  if (result != SP_OK) {
    ROS_ERROR_STREAM("Cannot get vendor ID and product ID." << result);
    return nullptr;
  }

  // These are the Piksi Multi USB identifiers. (see lsusb -v)
  const uint16_t kIDVendor = 0x2E69;
  const uint16_t kIDProduct = 0x1001;

  if (usb_vid == kIDVendor && usb_pid == kIDProduct) {
    // Get serial number as a unique identifier.
    return sp_get_port_usb_serial(port);
  } else {
    return nullptr;
  }
}

void DeviceUSB::printDeviceInfo(struct sp_port* port) {
  if (!port) return;

  ROS_INFO("Port: %s", sp_get_port_name(port));
  ROS_INFO("Manufacturer: %s", sp_get_port_usb_manufacturer(port));
  ROS_INFO("Product: %s", sp_get_port_usb_product(port));
  ROS_INFO("USB serial number: %s", sp_get_port_usb_serial(port));
  int usb_bus = -1;
  int usb_address = -1;
  sp_return result = sp_get_port_usb_bus_address(port, &usb_bus, &usb_address);
  if (result == SP_OK) {
    ROS_INFO("Bus: %d Adress: %d", usb_bus, usb_address);
  }
  int usb_vid = -1;
  int usb_pid = -1;
  result = sp_get_port_usb_vid_pid(port, &usb_vid, &usb_pid);
  if (result == SP_OK) {
    ROS_INFO("VendorID:ProductID: %4.0X:%4.0X", usb_vid, usb_pid);
  }
}

}  // namespace piksi_multi_cpp
