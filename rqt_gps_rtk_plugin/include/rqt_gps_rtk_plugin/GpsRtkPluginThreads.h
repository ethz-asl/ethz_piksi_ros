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

class UARTThread : public QThread {
 Q_OBJECT

 public:
  UARTThread() : QThread(), stop_thread_(true),
                 port_("/dev/ttyUSB0") {}

  // Important: Baudrate is not set as e.g. 115200, but as the constant (B115200)!!
  void setPort(const std::string &portname, const uint baudrate) {
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

  void createString(const SBP_MSG_OBS &msg, std::string *out) {
    std::map<uint8_t, std::string> code_map = {
        {0, "GPS L1CA"},
        {1, "GPS L2CM"},
        {2, "SBAS L1CA"},
        {3, "GLO L1CA"},
        {4, "GLO L2CA"},
        {5, "GPS L1P"},
        {6, "GPS L2P"},
        {12, "BDS2 B1"},
        {13, "BDS2 B2"},
        {14, "GAL E1B"},
        {20, "GAL E7I"}

    };

    std::stringstream sstream;
    sstream << "Correction Part " << msg.header.n_obs.index + 1 << " of " << msg.header.n_obs.total_n << std::endl;
    sstream << "GPS Week:\t\t " << msg.header.wn << std::endl;
    sstream << "Time of week:\t\t" << msg.header.tow << " [ms]" << std::endl;
    sstream << "Observations:\t\t" << msg.obs.size() << std::endl;
    sstream << "------------------------------------------\t\t" << std::endl;

    for (const SBP_MSG_OBS_OBSERVATION &obs : msg.obs) {
      if (code_map.count(obs.sid_code)) {
        sstream << code_map[obs.sid_code] << " / " << (int) obs.sid_sat << "\t";
      } else {
        sstream << "???? / " << (int) obs.sid_sat << "\t";
      }

      //add carrier noise density
      sstream << ((float) obs.cn0) / 4.0 << " dB Hz\t\t";

      if (obs.flags.RAIM_excl) {
        sstream << "!EXCLUDED! ";
      } else {
        sstream << "           ";
      }

      if (obs.flags.pseodorange_valid) {
        sstream << "PSDO ";
      } else {
        sstream << "     ";
      }

      if (obs.flags.carrier_phase_valid) {
        sstream << "CARR ";
      } else {
        sstream << "     ";
      }

      if (obs.flags.half_cycle_amb_resolv) {
        sstream << "CYCL ";
      } else {
        sstream << "     ";
      }

      if (obs.flags.doppler_valid) {
        sstream << "DPLR ";
      } else {
        sstream << "     ";
      }

      sstream << std::endl;
    }
    *out = sstream.str();
  }

  void run() override {
    stop_thread_ = false;
    fd_serial_port_ = open(port_.c_str(), O_RDWR);
    if (fd_serial_port_ != 1) {
      fcntl(fd_serial_port_, F_SETFL, 0);
      struct termios port_settings;      // structure to store the port settings in

      cfsetispeed(&port_settings, baudrate_);    // set baud rates
      cfsetospeed(&port_settings, baudrate_);

      port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
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
        size_t n = read(fd_serial_port_, &buf, 1);

        if (n == 1) {
          if (d.addToBuffer(buf) == SBP_MSG_TYPE::MSG_OBS) {

            // unpack message
            SBP_MSG_OBS msgObs;
            if (d.getCurrentMessage(&msgObs)) {
              std::string message;
              createString(msgObs, &message);
              emit resultReady(message.c_str());
            }
          }
        }

      } while (!stop_thread_);
    } else {
      emit resultReady("Could not open port.");
      exit(1);
      return;
    }

    exit(0);
  }

 signals:
  void resultReady(const QString &s);

};
#endif //RQT_GPS_RTK_PLUGIN_GPSRTKPLUGINTHREADS_H
