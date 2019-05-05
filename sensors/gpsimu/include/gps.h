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

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

using std::setprecision;
using std::string;
using std::vector;

using namespace GeographicLib;
// Define a UTM projection for an arbitrary ellipsoid
class UTMalt {
 private:
  GeographicLib::TransverseMercator _tm;  // The projection
  double _lon0;  // Central longitude  double _lon0;            // Central
                 // longitude

  double _falseeasting, _falsenorthing;

 public:
  UTMalt(double a,  // equatorial radius
         double f,  // flattening
         int zone,  // the UTM zone + hemisphere
         bool northp)
      : _tm(a, f, Constants::UTM_k0()),
        _lon0(6 * zone - 183),
        _falseeasting(5e5),
        _falsenorthing(northp ? 0 : 100e5) {
    if (!(zone >= 1 && zone <= 60)) throw GeographicErr("zone not in [1,60]");
  }
  void Forward(double lat, double lon, double& x, double& y) {
    _tm.Forward(_lon0, lat, lon, x, y);
    x += _falseeasting;
    y += _falsenorthing;
  }
  void Reverse(double x, double y, double& lat, double& lon) {
    x -= _falseeasting;
    y -= _falsenorthing;
    _tm.Reverse(_lon0, x, y, lat, lon);
  }
};

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
  Sleep(milliseconds);  // 100 ms
#else
  usleep(milliseconds * 1000);  // 100 ms
#endif
}

void enumerate_ports() {
  vector<serial::PortInfo> devices_found = serial::list_ports();

  vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while (iter != devices_found.end()) {
    serial::PortInfo device = *iter++;
  }
}

void print_usage() {
  std::cerr << "Usage: test_serial {-e|<serial port address>} ";
  std::cerr << "<baudrate> [test string]" << std::endl;
}

int run() {
  string port("/dev/ttyUSB0");
  enumerate_ports();
  unsigned long baud = 115200;

  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  /* cout << "Is the serial port open?";
   if (my_serial.isOpen())
     cout << " Yes." << endl;
   else
     cout << " No." << endl;*/

  struct gps_data {
    int date;   /*日期  */
    float time; /* gps定位时间 */
    //
    float heading; /*航向 */
    float pitch;   /*pitch*/
    float roll;    /*roll*/
    //
    double latitude;  /*纬度 */
    double longitude; /* 经度 */
    float altitude;   /*altitude*/
    //
    float speed_v; /* 速度 */
    float speed_n;
    float speed_u;
    //
    float basin_line;
    //
    int NSV1;
    int NSV2;
    //
    char status1;
    //
    char status2;
    //
    char check;
    //
    double UTM_x;
    //
    double UTM_y;
    // UINT check;
  } gps_data;

  // read data from the serial port
  while (1) {
    string buffer = my_serial.readline(200);

    string firstname(buffer.substr(0, 1));
    string strcompare = "$";

    if (firstname.compare(strcompare) == 0) {
      const char* bb = buffer.c_str();

      sscanf(bb, "$GPFPD,%d,%f,%f,%f,%f,%lf,%lf,%f,%f,%f,%f,%f,%d,%d,%2s*",
             &(gps_data.date),        // date
             &(gps_data.time),        // time
             &(gps_data.heading),     // heading
             &(gps_data.pitch),       // pitch
             &(gps_data.roll),        // roll
             &(gps_data.latitude),    // latitude
             &(gps_data.longitude),   // longitude
             &(gps_data.altitude),    // altitude
             &(gps_data.speed_v),     // speed_of_east
             &(gps_data.speed_n),     // speed_of_north
             &(gps_data.speed_u),     // speed_of_sky
             &(gps_data.basin_line),  // basin_line
             &(gps_data.NSV1),        // the_satellite_number_of_first
             &(gps_data.NSV2),        // the_satellite_number_of_second
             &(gps_data.status1)      // GPS_status
             /*&(gps_data.check)*/);  // CHECK

      UTMalt tm(6378388, 1 / 297.0, 51,
                true);  // International ellipsoid, zone 30n
      {
        // Sample forward calculation
        double lat = gps_data.latitude, lon = gps_data.longitude;  //
        double x, y;
        tm.Forward(lat, lon, x, y);
        gps_data.UTM_x = x;
        gps_data.UTM_y = y;
        // cout << fixed << setprecision(0) << x << " " << y << "\n";
      }

      std::cout << "buffer=    " << buffer << std::endl;
      std::cout << "date:      " << gps_data.date << std::endl;
      std::cout << "time:      " << std::fixed << setprecision(3)
                << gps_data.time << std::endl;
      std::cout << "heading:   " << std::fixed << setprecision(3)
                << gps_data.heading << std::endl;
      std::cout << "pitch:     " << std::fixed << setprecision(3)
                << gps_data.pitch << std::endl;
      std::cout << "roll:      " << std::fixed << setprecision(3)
                << gps_data.roll << std::endl;
      std::cout << "latitud:   " << std::fixed << setprecision(7)
                << gps_data.latitude << std::endl;
      std::cout << "longitude: " << std::fixed << setprecision(7)
                << gps_data.longitude << std::endl;
      std::cout << "UTM_x:     " << std::fixed << setprecision(7)
                << gps_data.UTM_x << std::endl;
      std::cout << "UTM_y:     " << std::fixed << setprecision(7)
                << gps_data.UTM_y << std::endl;
      std::cout << "altitude:  " << std::fixed << setprecision(2)
                << gps_data.altitude << std::endl;
      std::cout << "speed_v:   " << std::fixed << setprecision(3)
                << gps_data.speed_v << std::endl;
      std::cout << "speed_u:   " << std::fixed << setprecision(3)
                << gps_data.speed_u << std::endl;
      std::cout << "speed_n:   " << std::fixed << setprecision(3)
                << gps_data.speed_n << std::endl;
      std::cout << "basinLine  " << std::fixed << setprecision(3)
                << gps_data.basin_line << std::endl;
      std::cout << "NSV1:      " << gps_data.NSV1 << std::endl;
      std::cout << "NSV2:      " << gps_data.NSV2 << std::endl;
      switch (gps_data.status2) {
        case '0':
          std::cout << "Satus:     GPS初始化" << std::endl;
          break;
        case '1':
          std::cout << "Satus:     粗对准" << std::endl;
          break;
        case '2':
          std::cout << "Satus:     精对准" << std::endl;
          break;
        case '3':
          std::cout << "Satus:     GPS定位" << std::endl;
          break;
        case '4':
          std::cout << "Satus:     GPS定向" << std::endl;
          break;
        case '5':
          std::cout << "Satus:     GPS RTK" << std::endl;
          break;
        case 'B':
          std::cout << "Satus:     差分定向" << std::endl;
          break;
        default:
          std::cout << "Satus:     状态未知" << std::endl;
      }

    }
  }
  return 0;
}

#endif