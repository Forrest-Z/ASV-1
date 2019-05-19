/*
***********************************************************************
* gpsdata.h:
* header file to define the constant and real-time data in
* the GPS/IMU data
* This header file can be read by both C and C++ compilers
*
*  by Hu.ZH (CrossOcean.ai)
***********************************************************************
*/

#ifndef _GPSDATA_H_
#define _GPSDATA_H_

#include <Eigen/Core>
#include <Eigen/Dense>

// enum gpsstatus {
//   initialization = 0,       // 初始化
//   coarse_calibration = 1,   // 粗对准
//   precise_calibration = 2,  // 精对准
//   GPS_position = 3,         // GPS定位
//   GPS_heading = 4,          // GPS定向
//   RTK = 5,                  // RTK
//   DMI
// };

// real-time data from the gps/imu sensors
struct gpsRTdata {
  /**** GPFPD *****/
  int date;          // GPS week
  double time;       // GPS time (second)
  double heading;    // 航向角 0 ~ 359.99, 以真北为参考基准
  double pitch;      // 俯仰角 -90 ~ 90
  double roll;       // 横滚角 -180 ~ 180
  double latitude;   // 纬度 -90 ~ 90
  double longitude;  // 经度 -180 ~ 180
  double altitude;   // 高度 (m)
  double Ve;         // 东向速度 (m/s)
  double Vn;         // 北向速度 (m/s)
  double Vu;         // 天向速度 (m/s)
  double base_line;  // 基线长度 (m)
  int NSV1;          // # of satellites from antenna 1
  int NSV2;          // # of satellites from antenna 2
  char status;       // status
  char check[3];     // check
  /**** UTM projection   ****/
  double UTM_x;  // 投影的 x (m)
  double UTM_y;  // 投影的 y (m)
};

#endif /* _GPSDATA_H_ */