/*
 *************************************************
 *gps.h
 *function for read and transter the data to UTM
 *using serial lib to read the serial data
 *using GeographicLib to transfer to UTM
 *the header file can be read by C++ compilers
 *
 *by Zhenqiu Fu
 *************************************************
 */

#ifndef __GPS_H__
#define __GPS_H__

#include <string.h>
#include <GeographicLib/TransverseMercator.hpp>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include "gpsdata.h"
#include "serial/serial.h"

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
  Sleep(milliseconds);  // 100 ms
#else
  usleep(milliseconds * 1000);  // 100 ms
#endif
}

class gpsimu {
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

  gpsimu& gpsonestep() {
    //
    std::string t_serial_buffer("gps error");
    t_serial_buffer = GPS_serial.readline(200);

    std::size_t pos = t_serial_buffer.find("$GPFPD");
    if (pos != std::string::npos) {
      serial_buffer = t_serial_buffer.substr(pos);
      sscanf(
          serial_buffer.c_str(),
          "$GPFPD,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%s%s*",
          &(gps_data.date),        // date
          &(gps_data.time),        // time
          &(gps_data.heading),     // heading
          &(gps_data.pitch),       // pitch
          &(gps_data.roll),        // roll
          &(gps_data.latitude),    // latitude
          &(gps_data.longitude),   // longitude
          &(gps_data.altitude),    // altitude
          &(gps_data.Ve),          // speed_of_east
          &(gps_data.Vn),          // speed_of_north
          &(gps_data.Vu),          // speed_of_sky
          &(gps_data.base_line),   // basin_line
          &(gps_data.NSV1),        // the_satellite_number_of_first
          &(gps_data.NSV2),        // the_satellite_number_of_second
          &(gps_data.status1),     // GPS_status1
          &(gps_data.status2)      // GPS_status2
          /*&(gps_data.check)*/);  // CHECK
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

#endif