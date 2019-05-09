/*
*******************************************************************************
* testGPS.cc:
* unit test for serial communication and UTM projection for GPS/IMU
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <cstdio>
#include <thread>
#include "gps.h"
using std::setprecision;
int main() {
  // real time GPS/IMU data
  gpsRTdata gps_data{
      0,                // date
      0,                // time
      0,                // heading
      0,                // pitch
      0,                // roll
      0,                // latitude
      0,                // longitude
      0,                // altitude
      0,                // Ve
      0,                // Vn
      0,                // Vu
      0,                // base_line
      0,                // NSV1
      0,                // NSV2
      'a',              // status
      {'a', 'b', '0'},  // check
      0,                // UTM_x
      0                 // UTM_y
  };
  try {
    gpsimu _gpsimu(51, true, 115200);  // zone 30n

    while (1) {
      gps_data = _gpsimu.gpsonestep().getgpsRTdata();
      std::cout << _gpsimu;
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}
