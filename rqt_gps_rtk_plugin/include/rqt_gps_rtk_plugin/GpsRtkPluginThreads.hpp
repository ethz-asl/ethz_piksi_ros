//
// Created by mpantic on 07.03.19.
//

#ifndef RQT_GPS_RTK_PLUGIN_GPSRTKPLUGINTHREADS_H
#define RQT_GPS_RTK_PLUGIN_GPSRTKPLUGINTHREADS_H
#include <QThread>
#include <rqt_gps_rtk_plugin/SBPStreamDecoder.hpp>
#include <atomic>

//serial port
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <sstream>

// udp
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

// Thread to read corrections from UDP

class UDPThread : public QThread {
 Q_OBJECT

 public:
  UDPThread() : QThread(),
                fd_socket_(0),
                stop_thread_(true),
                udp_port_(26078) {}

  void stop() {
    stop_thread_ = true;
  }

 private:
  int fd_socket_;
  std::atomic_bool stop_thread_;
  uint udp_port_;

  void run() override {
    stop_thread_ = false;

    // adress information
    struct addrinfo hints, * res;
    socklen_t fromlen;
    struct sockaddr_storage addr;

    // set port
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;  // use IPv4 or IPv6, whichever
    hints.ai_socktype = SOCK_DGRAM; // get full datagramm without IP header.
    hints.ai_flags = AI_PASSIVE;
    getaddrinfo(nullptr, std::to_string(udp_port_).c_str(), &hints, &res);

    // get socket & bind
    fd_socket_ = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (bind(fd_socket_, res->ai_addr, res->ai_addrlen) == 0) {

      std::vector<uint8_t> buffer;
      buffer.resize(1600); // larger than largest udp packets usually are.

      // receive data.
      do {
        ssize_t received_length;
        if ((received_length = recvfrom(fd_socket_,
                                        buffer.data(),
                                        buffer.size(),
                                        0,
                                        (sockaddr*) &addr,
                                        &fromlen))) {
          // get message
          SBP_MSG_OBS msg;
          if (SBPDecoder::decode<SBP_MSG_OBS>(buffer, &msg)) {
            resultReady(msg.str().c_str());
          }
        }

      } while (!stop_thread_);

      close(fd_socket_);

    } else {
      resultReady("Socket error");
      exit(1);
      return;
    }

  }

 signals:
  void resultReady(const QString& s);

};

// Thread to read corrections from UART
class UARTThread : public QThread {
 Q_OBJECT

 public:
  UARTThread() : QThread(),
                 fd_serial_port_(-1),
                 stop_thread_(true),
                 port_("/dev/ttyUSB0"),
                 baudrate_(B115200) {}

  // Important: Baudrate is not set as e.g. 115200,
  // but as the constant (B115200)!!
  void setPort(const std::string& portname, const uint baudrate) {
    port_ = portname;
    baudrate_ = baudrate;
  }

  void stop() {
    stop_thread_ = true;
  }

 private:
  int fd_serial_port_;
  std::atomic_bool stop_thread_;
  std::string port_;
  uint baudrate_;

  void run() override {
    stop_thread_ = false;
    fd_serial_port_ = open(port_.c_str(), O_RDWR);
    if (fd_serial_port_ != 1) {
      fcntl(fd_serial_port_, F_SETFL, 0);
      struct termios port_settings;

      cfsetispeed(&port_settings, baudrate_);    // set baud rates
      cfsetospeed(&port_settings, baudrate_);

      port_settings.c_cflag &=
          ~PARENB;    // set no parity, stop bits, data bits
      port_settings.c_cflag &= ~CSTOPB;
      port_settings.c_cflag &= ~CSIZE;
      port_settings.c_cflag |= CS8;

      cfmakeraw(&port_settings);

      // apply the settings to the port
      if (tcsetattr(fd_serial_port_, TCSANOW, &port_settings) != 0) {
        emit resultReady("Failed to set attributes.");
        exit(1);
        return;
      }

      tcflush(fd_serial_port_, TCIFLUSH);
      SBPStreamDecoder d(300, {SBP_MSG_TYPE::MSG_OBS});

      do {
        uint8_t buf;
        ssize_t n = read(fd_serial_port_, &buf, 1);

        if (n == 1) {
          if (d.addToBuffer(buf) == SBP_MSG_TYPE::MSG_OBS) {

            // unpack message
            SBP_MSG_OBS msgObs;
            if (d.getCurrentMessage(&msgObs)) {
              emit resultReady(msgObs.str().c_str());
            }
          }
        }

      } while (!stop_thread_);
      close(fd_serial_port_);
    } else {
      emit resultReady("Could not open port.");
      exit(1);
      return;
    }

    exit(0);
  }

 signals:
  void resultReady(const QString& s);

};
#endif //RQT_GPS_RTK_PLUGIN_GPSRTKPLUGINTHREADS_H
