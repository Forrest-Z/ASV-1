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
#include "motorclientdata.h"
#include "plannerdata.h"
#include "priority.h"
#include "serial/serial.h"
#include "timecounter.h"

template <int m, int n = 3>
class guiserver {
  template <int _m, int _n>
  friend std::ostream &operator<<(std::ostream &, const guiserver<_m, _n> &);

 public:
  guiserver(unsigned long _rate = 19200,
            const std::string &_port = "/dev/ttyUSB0")
      : my_serial(_port, _rate, serial::Timeout::simpleTimeout(500)),
        send_buffer(""),
        recv_buffer(""),
        gui_connetion_failure_count(0) {
    judgeserialstatus();
  }
  ~guiserver() {}

  void guicommunication(indicators &_indicators,
                        const estimatorRTdata &_estimatorRTdata,
                        const plannerRTdata &_plannerRTdata,
                        const gpsRTdata &_gpsRTdata,
                        const motorRTdata<m> &_motorRTdata) {
    checkguiconnection(_indicators);
    // senddata2gui(_indicators, _controllerRTdata, _estimatorRTdata,
    //              _plannerRTdata, _gpsRTdata, _motorRTdata);
    senddata2gui(_indicators, _estimatorRTdata, _plannerRTdata, _gpsRTdata,
                 _motorRTdata);
    parsedatafromgui(_indicators);
  }

 private:
  serial::Serial my_serial;
  std::string send_buffer;
  std::string recv_buffer;

  int gui_connetion_failure_count;
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

  void checkguiconnection(indicators &_indicators) {
    if (gui_connetion_failure_count > 10)
      _indicators.gui_connection = 0;
    else
      _indicators.gui_connection = 1;
  }
  // convert real time GPS data to sql string
  void convert2string(const indicators &_indicators, std::string &_str) {
    _str += ",";
    _str += std::to_string(_indicators.gui_connection);
    _str += ",";
    _str += std::to_string(_indicators.joystick_connection);
    _str += ",";
    _str += std::to_string(_indicators.indicator_controlmode);
    _str += ",";
    _str += std::to_string(_indicators.indicator_windstatus);
  }
  // convert real time GPS data to sql string
  void convert2string(const gpsRTdata &_gpsRTdata, std::string &_str) {
    _str += ",";
    _str += std::to_string(_gpsRTdata.latitude);
    _str += ",";
    _str += std::to_string(_gpsRTdata.longitude);
  }

  // convert real time motor data to sql string
  void convert2string(const motorRTdata<m> &_motorRTdata, std::string &_str) {
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_motorRTdata.feedback_alpha[i]);
    }
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_motorRTdata.feedback_rotation[i]);
    }
    for (int i = 0; i != (2 * m); ++i) {
      _str += ",";
      _str += std::to_string(_motorRTdata.feedback_torque[i]);
    }
    _str += ",";
    _str += std::to_string(_motorRTdata.feedback_allinfo);
  }
  void convert2string(const controllerRTdata<m, n> &_RTdata,
                      std::string &_str) {
    // the angle of each propeller (command)
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.alpha_deg(i));
    }
    // the speed of each propeller (command)
    for (int i = 0; i != m; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.rotation(i));
    }
  }
  void convert2string(const estimatorRTdata &_RTdata, std::string &_str) {
    // State
    for (int i = 0; i != 6; ++i) {
      _str += ",";
      _str += std::to_string(_RTdata.State(i));
    }
    // Roll and Pitch
    _str += ",";
    _str += std::to_string(_RTdata.motiondata_6dof(3));
    _str += ",";
    _str += std::to_string(_RTdata.motiondata_6dof(4));
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

  void parsedatafromgui(indicators &_indicators) {
    recv_buffer = my_serial.readline(40, "\n");

    std::size_t pos = recv_buffer.find("$IPAC");
    if (pos != std::string::npos) {
      recv_buffer = recv_buffer.substr(pos);
      int _controlmode = 0;
      int _windindicator = 0;
      double wp1 = 0.0;
      double wp2 = 0.0;
      double wp3 = 0.0;
      sscanf(recv_buffer.c_str(), "$IPAC,%d,%d,%lf,%lf,%lf",
             &_controlmode,    // date
             &_windindicator,  // time
             &wp1,             // waypoint1
             &wp2,             // waypoint2
             &wp3              // waypoint3
      );
      _indicators.indicator_controlmode = _controlmode;
      _indicators.indicator_windstatus = _windindicator;
      gui_connetion_failure_count = 0;
    } else {
      recv_buffer = "error";
      ++gui_connetion_failure_count;
    }
  }

  void senddata2gui(const indicators &_indicators,
                    const controllerRTdata<m, n> &_controllerRTdata,
                    const estimatorRTdata &_estimatorRTdata,
                    const plannerRTdata &_plannerRTdata,
                    const gpsRTdata &_gpsRTdata,
                    const motorRTdata<m> &_motorRTdata) {
    send_buffer.clear();
    send_buffer = "$IPAC";
    convert2string(_indicators, send_buffer);
    convert2string(_gpsRTdata, send_buffer);
    convert2string(_estimatorRTdata, send_buffer);
    convert2string(_controllerRTdata, send_buffer);
    convert2string(_motorRTdata, send_buffer);
    convert2string(_plannerRTdata, send_buffer);

    send_buffer += "\n";
    size_t bytes_wrote = my_serial.write(send_buffer);
    std::cout << bytes_wrote << std::endl;
  }

  void senddata2gui(const indicators &_indicators,
                    const estimatorRTdata &_estimatorRTdata,
                    const plannerRTdata &_plannerRTdata,
                    const gpsRTdata &_gpsRTdata,
                    const motorRTdata<m> &_motorRTdata) {
    send_buffer.clear();
    send_buffer = "$IPAC";
    convert2string(_indicators, send_buffer);
    convert2string(_gpsRTdata, send_buffer);
    convert2string(_estimatorRTdata, send_buffer);
    convert2string(_motorRTdata, send_buffer);
    convert2string(_plannerRTdata, send_buffer);

    send_buffer += "\n";
    size_t bytes_wrote = my_serial.write(send_buffer);
    std::cout << bytes_wrote << std::endl;
  }
};

template <int _m, int _n>
std::ostream &operator<<(std::ostream &os,
                         const guiserver<_m, _n> &_guiserver) {
  os << "Buffer sent is: " << _guiserver.send_buffer;
  os << "Buffer recv is: " << _guiserver.recv_buffer << std::endl;
  return os;
}

#endif /* _GUISERVER_H_ */