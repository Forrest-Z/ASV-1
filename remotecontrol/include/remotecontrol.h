/*
 *************************************************
 *remotecontrol.h
 *function for receive control message from remote controller
 *the header file can be read by C++ compilers
 *
 *by Hu Z.H. (SJTU)
 *************************************************
 */

#ifndef _REMOTECONTROL_H_
#define _REMOTECONTROL_H_

#include <string.h>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include "priority.h"
#include "remotecontroldata.h"
#include "serial/serial.h"

class remotecontrol {
 public:
  explicit remotecontrol(unsigned long _baud,  // baudrate
                         const std::string& _port = "/dev/ttyUSB0")
      : rc_serial(_port, _baud, serial::Timeout::simpleTimeout(2000)),
        rc_connetion_failure_count(0) {}
  remotecontrol() = delete;
  ~remotecontrol() {}
  // read serial data
  remotecontrol& readserialdata(recontrolRTdata& _recontrolRTdata) {
    serial_buffer = rc_serial.readline(200);
    std::size_t pos = serial_buffer.find("PPM1");
    float _ppm1 = 0.0;
    float _ppm2 = 0.0;
    float _ppm3 = 0.0;
    float _ppm4 = 0.0;
    float _ppm5 = 0.0;
    float _ppm6 = 0.0;
    float _ppm7 = 0.0;
    float _ppm8 = 0.0;
    if (pos != std::string::npos) {
      serial_buffer = serial_buffer.substr(pos);
      sscanf(serial_buffer.c_str(),
             "PPM1-01=%f   PPM1-02=%f   PPM1-03=%f   "
             "PPM1-04=%f   PPM1-05=%f   PPM1-06=%f   "
             "PPM1-07=%f   PPM1-08=%f",
             &_ppm1,  //
             &_ppm2,  //
             &_ppm3,  //
             &_ppm4,  //
             &_ppm5,  //
             &_ppm6,  //
             &_ppm7,  //
             &_ppm8   //
      );
      rc_connetion_failure_count = 0;
    } else {
      ++rc_connetion_failure_count;
    }

    _recontrolRTdata.right_joystick_LR = linearconversion(_ppm1);
    _recontrolRTdata.right_joystick_UD = linearconversion(_ppm2);
    _recontrolRTdata.left_joystick_UD = linearconversion(_ppm3);
    _recontrolRTdata.left_joystick_LR = linearconversion(_ppm4);
    _recontrolRTdata.SA = linearconversion(_ppm5);
    _recontrolRTdata.SB = linearconversion(_ppm6);
    _recontrolRTdata.SC = linearconversion(_ppm7);
    _recontrolRTdata.SD = linearconversion(_ppm8);

    return *this;
  }

  void parsercdata(indicators& _indicators, Eigen::Vector3d& _command,
                   const Eigen::Matrix<double, 3, 2>& _command_limit,
                   const recontrolRTdata& _recontrolRTdata) {
    if (_recontrolRTdata.right_joystick_UD > 0)
      _command(0) =
          _command_limit(0, 0) * _recontrolRTdata.right_joystick_UD / 100.0;
    else
      _command(0) =
          -_command_limit(0, 1) * _recontrolRTdata.right_joystick_UD / 100.0;

    if (_recontrolRTdata.right_joystick_LR > 0)
      _command(1) =
          _command_limit(1, 0) * _recontrolRTdata.right_joystick_LR / 100.0;
    else
      _command(1) =
          -_command_limit(1, 1) * _recontrolRTdata.right_joystick_LR / 100.0;

    if (_recontrolRTdata.left_joystick_LR > 0)
      _command(2) =
          _command_limit(2, 0) * _recontrolRTdata.left_joystick_LR / 100.0;
    else
      _command(2) =
          -_command_limit(2, 1) * _recontrolRTdata.left_joystick_LR / 100.0;

    if (_recontrolRTdata.SA > 90) {
      _indicators.indicator_controlmode = 0;  // manual
    } else if (_recontrolRTdata.SA < -90) {
      if (_recontrolRTdata.SB > 90)
        _indicators.indicator_controlmode = 2;  // manuveuring
      else if (_recontrolRTdata.SB < -90)
        _indicators.indicator_controlmode = 3;  // DP
      else
        _indicators.indicator_controlmode = 1;  // autopilot
    } else {
    }
  }

  std::string getserialbuffer() const { return serial_buffer; }
  int checkrcconnection() const {
    if (rc_connetion_failure_count > 10) return 0;
    return 1;
  }

 private:
  // serial data
  serial::Serial rc_serial;
  std::string serial_buffer;
  int rc_connetion_failure_count;

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
    }
  }
  // convert 1100 ~ 1940 to -100 ~ 100
  float linearconversion(float _x) { return 0.2381 * _x - 361.9; }
};

#endif