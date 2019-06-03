/*
 *************************************************
 *gps.h
 *function for read and transter the data to UTM
 *using serial lib to read the serial data
 *using GeographicLib to transfer to UTM
 *the header file can be read by C++ compilers
 *
 *by Zhenqiu Fu (SJTU)
 *************************************************
 */

#ifndef _GPS_H_
#define _GPS_H_

#include <string.h>
#include <GeographicLib/TransverseMercator.hpp>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include "gpsdata.h"
#include "serial/serial.h"

class gpsimu {
  friend std::ostream& operator<<(std::ostream&, const gpsimu&);

 public:
  explicit gpsimu(int _zone,            // the UTM zone
                  bool _northp,         // hemisphere,
                  unsigned long _baud,  // baudrate
                  const std::string& _port = "/dev/ttyUSB0")
      : GPS_serial(_port, _baud, serial::Timeout::simpleTimeout(2000)),
        _tm(6378388,    // equatorial radius
            1 / 297.0,  // flattening
            GeographicLib::Constants::UTM_k0()),
        _lon0(6 * _zone - 183),
        _falseeasting(5e5),
        _falsenorthing(_northp ? 0 : 100e5) {
    if (!(_zone >= 1 && _zone <= 60))
      throw GeographicLib::GeographicErr("zone not in [1,60]");
  }
  gpsimu() = delete;
  ~gpsimu() {}
  // read serial data and transform to UTM
  gpsimu& gpsonestep() {
    std::string t_serial_buffer("gps error");
    t_serial_buffer = GPS_serial.readline(200);

    std::size_t pos = t_serial_buffer.find("$GPFPD");
    if (pos != std::string::npos) {
      serial_buffer = t_serial_buffer.substr(pos);
      sscanf(
          serial_buffer.c_str(),
          "$GPFPD,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,0%s%s",
          &(gps_data.date),       // date
          &(gps_data.time),       // time
          &(gps_data.heading),    // heading
          &(gps_data.pitch),      // pitch
          &(gps_data.roll),       // roll
          &(gps_data.latitude),   // latitude
          &(gps_data.longitude),  // longitude
          &(gps_data.altitude),   // altitude
          &(gps_data.Ve),         // speed_of_east
          &(gps_data.Vn),         // speed_of_north
          &(gps_data.Vu),         // speed_of_sky
          &(gps_data.base_line),  // basin_line
          &(gps_data.NSV1),       // the_satellite_number_of_first
          &(gps_data.NSV2),       // the_satellite_number_of_second
          &(gps_data.status),     // GPS_status
          gps_data.check          // check
      );
      Forward(gps_data.latitude, gps_data.longitude, gps_data.UTM_x,
              gps_data.UTM_y);
    } else
      serial_buffer = t_serial_buffer;

    return *this;
  }

  gpsRTdata getgpsRTdata() const { return gps_data; }
  std::string getserialbuffer() const { return serial_buffer; }

 private:
  // serial data
  serial::Serial GPS_serial;
  std::string serial_buffer;
  // GeographicLib data
  // Define a UTM projection for an arbitrary ellipsoid
  GeographicLib::TransverseMercator _tm;  // The projection
  double _lon0;                           // Central longitude
  double _falseeasting, _falsenorthing;

  //
  gpsRTdata gps_data;

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
    }
  }
  // convert longitude and latitude to UTM
  void Forward(double lat, double lon, double& x, double& y) {
    _tm.Forward(_lon0, lat, lon, x, y);
    x += _falseeasting;
    y += _falsenorthing;
  }
  // convert UTM to longitude and latitude
  void Reverse(double x, double y, double& lat, double& lon) {
    x -= _falseeasting;
    y -= _falsenorthing;
    _tm.Reverse(_lon0, x, y, lat, lon);
  }
};

std::ostream& operator<<(std::ostream& os, const gpsimu& _gps) {
  // buffer = _gpsimu.getserialbuffer();
  os << "buffer=    " << _gps.serial_buffer;
  os << "date:      " << _gps.gps_data.date << std::endl;
  os << "time:      " << std::fixed << std::setprecision(4)
     << _gps.gps_data.time << std::endl;
  os << "heading:   " << std::fixed << std::setprecision(3)
     << _gps.gps_data.heading << std::endl;
  os << "pitch:     " << std::fixed << std::setprecision(3)
     << _gps.gps_data.pitch << std::endl;
  os << "roll:      " << std::fixed << std::setprecision(3)
     << _gps.gps_data.roll << std::endl;
  os << "latitud:   " << std::fixed << std::setprecision(7)
     << _gps.gps_data.latitude << std::endl;
  os << "longitude: " << std::fixed << std::setprecision(7)
     << _gps.gps_data.longitude << std::endl;
  os << "UTM_x:     " << std::fixed << std::setprecision(7)
     << _gps.gps_data.UTM_x << std::endl;
  os << "UTM_y:     " << std::fixed << std::setprecision(7)
     << _gps.gps_data.UTM_y << std::endl;
  os << "altitude:  " << std::fixed << std::setprecision(2)
     << _gps.gps_data.altitude << std::endl;
  os << "speed_v:   " << std::fixed << std::setprecision(3) << _gps.gps_data.Ve
     << std::endl;
  os << "speed_u:   " << std::fixed << std::setprecision(3) << _gps.gps_data.Vn
     << std::endl;
  os << "speed_n:   " << std::fixed << std::setprecision(3) << _gps.gps_data.Vu
     << std::endl;
  os << "basinLine  " << std::fixed << std::setprecision(3)
     << _gps.gps_data.base_line << std::endl;
  os << "NSV1:      " << _gps.gps_data.NSV1 << std::endl;
  os << "NSV2:      " << _gps.gps_data.NSV2 << std::endl;
  printf("status: %c\n", _gps.gps_data.status);
  printf("check: %s\n", _gps.gps_data.check);

  switch (_gps.gps_data.status) {
    case '0':
      os << "Satus:     GPS初始化" << std::endl;
      break;
    case '1':
      os << "Satus:     粗对准" << std::endl;
      break;
    case '2':
      os << "Satus:     精对准" << std::endl;
      break;
    case '3':
      os << "Satus:     GPS定位" << std::endl;
      break;
    case '4':
      os << "Satus:     GPS定向" << std::endl;
      break;
    case '5':
      os << "Satus:     GPS RTK" << std::endl;
      break;
    case 'B':
      os << "Satus:     差分定向" << std::endl;
      break;
    default:
      os << "Satus:     状态未知" << std::endl;
  }
  os << std::endl;

  return os;
}

#endif