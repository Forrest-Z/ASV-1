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
#include "remotecontroldata.h"
#include "serial/serial.h"

class remotecontrol {
 public:
  explicit remotecontrol(unsigned long _baud,  // baudrate
                         const std::string& _port = "/dev/ttyUSB0")
      : rc_serial(_port, _baud, serial::Timeout::simpleTimeout(2000)) {}
  remotecontrol() = delete;
  ~remotecontrol() {}
  // read serial data and transform to UTM
  void rconestep(recontrolRTdata& _recontrolRTdata) {
    serial_buffer = rc_serial.readline(200);
    float _ppm1 = 0.0;
    float _ppm2 = 0.0;
    float _ppm3 = 0.0;
    float _ppm4 = 0.0;
    float _ppm5 = 0.0;
    float _ppm6 = 0.0;
    float _ppm7 = 0.0;
    float _ppm8 = 0.0;
    sscanf(serial_buffer.c_str(),
           "PPM1-01=%f   PPM1-02=%f   PPM1-03=%f   "
           "PPM1-04=%f   PPM1-05=%f   PPM1-06=%f  "
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
    _recontrolRTdata.ppm1 = static_cast<int>(_ppm1);
    _recontrolRTdata.ppm2 = static_cast<int>(_ppm2);
    _recontrolRTdata.ppm3 = static_cast<int>(_ppm3);
    _recontrolRTdata.ppm4 = static_cast<int>(_ppm4);
    _recontrolRTdata.ppm5 = static_cast<int>(_ppm5);
    _recontrolRTdata.ppm6 = static_cast<int>(_ppm6);
    _recontrolRTdata.ppm7 = static_cast<int>(_ppm7);
    _recontrolRTdata.ppm8 = static_cast<int>(_ppm8);
  }

  std::string getserialbuffer() const { return serial_buffer; }

 private:
  // serial data
  serial::Serial rc_serial;
  std::string serial_buffer;

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
    }
  }
};

#endif