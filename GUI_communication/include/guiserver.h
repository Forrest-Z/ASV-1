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

#include <boost/lexical_cast.hpp>
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
      : my_serial("/dev/ttyUSB1", 19200, serial::Timeout::simpleTimeout(500)),
        send_buffer(""),
        recv_buffer("") {
    judgeserialstatus();
  }
  ~guiserver() {}

  void guicommunication(const controllerRTdata<m, n> &_controllerRTdata,
                        const estimatorRTdata &_estimatorRTdata,
                        const plannerRTdata &_plannerRTdata,
                        const gpsRTdata &_gpsRTdata) {
    timecounter _timer;
    senddata2gui(_controllerRTdata, _estimatorRTdata, _plannerRTdata,
                 _gpsRTdata);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(500 - _timer.timeelapsed()));
    parsedatafromgui();
  }

 private:
  serial::Serial my_serial;
  std::string send_buffer;
  std::string recv_buffer;
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
    _str += ",";
    _str += std::to_string(_gpsRTdata.date);
    _str += ",";
    _str += std::to_string(_gpsRTdata.time);
    _str += ",";
    _str += std::to_string(_gpsRTdata.heading);
    _str += ",";
    _str += std::to_string(_gpsRTdata.pitch);
    _str += ",";
    _str += std::to_string(_gpsRTdata.roll);
    _str += ",";
    _str += boost::lexical_cast<std::string>(_gpsRTdata.latitude);
    _str += ",";
    _str += boost::lexical_cast<std::string>(_gpsRTdata.longitude);
    _str += ",";
    _str += std::to_string(_gpsRTdata.altitude);
    _str += ",";
    _str += std::to_string(_gpsRTdata.Ve);
    _str += ",";
    _str += std::to_string(_gpsRTdata.Vn);
    _str += ",";
    _str += std::to_string(_gpsRTdata.Vu);
    _str += ",";
    _str += std::to_string(_gpsRTdata.base_line);
    _str += ",";
    _str += std::to_string(_gpsRTdata.NSV1);
    _str += ",";
    _str += std::to_string(_gpsRTdata.NSV2);
    _str += ",";
    _str += std::to_string(_gpsRTdata.UTM_x);
    _str += ",";
    _str += std::to_string(_gpsRTdata.UTM_y);
  }
  void convert2string(const controllerRTdata<m, n> &_RTdata,
                      std::string &_str) {
    // the angle of each propeller
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.alpha_deg(i));
    }
    // the speed of each propeller
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.rotation(i));
    }
  }
  void convert2string(const estimatorRTdata &_RTdata, std::string &_str) {
    // Measurement
    for (int i = 0; i != 6; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.Measurement(i));
    }
    // State
    for (int i = 0; i != 6; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.State(i));
    }
  }

  void convert2string(const plannerRTdata &_RTdata, std::string &_str) {
    // setpoint
    for (int i = 0; i != 3; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.setpoint(i));
    }
    // v_setpoint
    for (int i = 0; i != 3; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.v_setpoint(i));
    }
  }

  void parsedatafromgui() {
    recv_buffer = my_serial.readline(40, "\n");

    int _controlmode = 0;
    int _windindicator = 0;
    double test_C = 0.0;
    std::size_t pos = recv_buffer.find("$IPAC");
    if (pos != std::string::npos) {
      recv_buffer = recv_buffer.substr(pos);
      sscanf(recv_buffer.c_str(), "$IPAC,%d,%d,%lf",
             &_controlmode,    // date
             &_windindicator,  // time
             &test_C           // heading
      );

    } else
      recv_buffer = "error";
  }

  void senddata2gui(const controllerRTdata<m, n> &_controllerRTdata,
                    const estimatorRTdata &_estimatorRTdata,
                    const plannerRTdata &_plannerRTdata,
                    const gpsRTdata &_gpsRTdata) {
    send_buffer.clear();
    static int i = 0;
    send_buffer = "$IPAC" + std::to_string(++i);
    convert2string(_gpsRTdata, send_buffer);
    convert2string(_controllerRTdata, send_buffer);
    convert2string(_estimatorRTdata, send_buffer);
    convert2string(_plannerRTdata, send_buffer);
    send_buffer += "\n";
    size_t bytes_wrote = my_serial.write(send_buffer);
  }
};

template <int _m, int _n>
std::ostream &operator<<(std::ostream &os,
                         const guiserver<_m, _n> &_guiserver) {
  os << "Buffer sent is: " << _guiserver.send_buffer;
  os << "Buffer recv is: " << _guiserver.recv_buffer << std::endl;
}

#endif /* _GUISERVER_H_ */