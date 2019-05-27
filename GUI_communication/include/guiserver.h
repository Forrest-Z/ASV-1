/*
***********************************************************************
* guiserver.h: communication for gui server
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _GUISERVER_H_
#define _GUISERVER_H_

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "controllerdata.h"
#include "easylogging++.h"
#include "estimatordata.h"
#include "gpsdata.h"
#include "plannerdata.h"
#include "serial/serial.h"
#include "timecounter.h"

template <int m, int n = 3>
class guiserver {
  template <int _m, int _n>
  friend std::ostream &operator<<(std::ostream &, const guiserver<_m, _n> &);

 public:
  guiserver()
      : my_serial("/dev/ttyUSB0", 19200, serial::Timeout::simpleTimeout(500)),
        send_buffer(""),
        recv_buffer(""),
        count(0) {
    judgeserialstatus();
  }
  ~guiserver() {}

  void convertalldata2string(const controllerRTdata<m, n> &_controllerRTdata,
                             const estimatorRTdata &_estimatorRTdata,
                             const plannerRTdata &_plannerRTdata,
                             const gpsRTdata &_gpsRTdata) {
    send_buffer.clear();
    recv_buffer.clear();
    send_buffer = "$TEST" + std::to_string(++count);
    convert2string(_controllerRTdata, send_buffer);
    convert2string(_estimatorRTdata, send_buffer);
    convert2string(_plannerRTdata, send_buffer);
    convert2string(_gpsRTdata, send_buffer);
    send_buffer += "\n";

    size_t bytes_wrote = my_serial.write(send_buffer);
    my_serial.flush();
    if (my_serial.waitReadable()) recv_buffer = my_serial.readline(20);
    my_serial.flush();
  }

  void convertalldata2string(const controllerRTdata<m, n> &_controllerRTdata,
                             const estimatorRTdata &_estimatorRTdata,
                             const plannerRTdata &_plannerRTdata) {
    send_buffer.clear();
    recv_buffer.clear();
    send_buffer = "$TEST" + std::to_string(++count);
    convert2string(_controllerRTdata, send_buffer);
    convert2string(_estimatorRTdata, send_buffer);
    convert2string(_plannerRTdata, send_buffer);
    send_buffer += "\n";

    size_t bytes_wrote = my_serial.write(send_buffer);
    if (my_serial.waitReadable()) recv_buffer = my_serial.readline(20);
  }

 private:
  serial::Serial my_serial;
  std::string send_buffer;
  std::string recv_buffer;
  int count;
  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
      std::cout << device.port.c_str() << ", " << device.description.c_str()
                << ", " << device.hardware_id.c_str() << std::endl;
    }
  }

  void judgeserialstatus() {
    if (my_serial.isOpen())
      CLOG(INFO, "gui-serial") << " serial port open successful!";
    else
      CLOG(INFO, "gui-serial") << " serial port open failure!";
  }

  // convert real time GPS data to sql string
  void convert2string(const gpsRTdata &_gpsRTdata, std::string &_str) {
    _str += ", ";
    _str += std::to_string(_gpsRTdata.date);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.time);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.heading);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.pitch);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.roll);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.latitude);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.longitude);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.altitude);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.Ve);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.Vn);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.Vu);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.base_line);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.NSV1);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.NSV2);
    _str += ", '";
    _str += std::string(1, _gpsRTdata.status);
    _str += "' , '";
    _str += std::string(_gpsRTdata.check);
    _str += "' , ";
    _str += std::to_string(_gpsRTdata.UTM_x);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.UTM_y);
  }
  void convert2string(const controllerRTdata<m, n> &_RTdata,
                      std::string &_str) {
    // the angle of each propeller
    for (int i = 0; i != m; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.alpha_deg(i));
    }
    // the speed of each propeller
    for (int i = 0; i != m; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.rotation(i));
    }
  }
  void convert2string(const estimatorRTdata &_RTdata, std::string &_str) {
    // Measurement
    for (int i = 0; i != 6; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.Measurement(i));
    }
    // State
    for (int i = 0; i != 6; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.State(i));
    }
  }

  void convert2string(const plannerRTdata &_RTdata, std::string &_str) {
    // setpoint
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.setpoint(i));
    }
    // v_setpoint
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.v_setpoint(i));
    }
  }
};

template <int _m, int _n>
std::ostream &operator<<(std::ostream &os,
                         const guiserver<_m, _n> &_guiserver) {
  os << "Buffer sent is: " << _guiserver.send_buffer;
  os << "Buffer recv is: " << _guiserver.recv_buffer << std::endl;
}

#endif /* _GUISERVER_H_ */