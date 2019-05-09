/*
***********************************************************************
* windcompensation.h: compute the wind load on USV using wind
* coefficient
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _WINDCOMPENSATION_H_
#define _WINDCOMPENSATION_H_

#include "controllerdata.h"

class windcompensation {
 public:
  windcompensation() {}
  // TODO
  Eigen::Vector2d convertbwind2global(const Eigen::Vector2d &_windbody,
                                      double v_heading, double v_speed) {
    Eigen::Vector2d wind_global = Eigen::Vector2d::Zero();
    return wind_global;
  }

  // TODO
  Eigen::Vector3d getwindload() { return load; }

 private:
  // Fx, Fy, Mz (wind force) in the body coordinate
  Eigen::Vector3d load;
  // wind direction and speed in body
  Eigen::Vector2d wind_body;  // direction, speed
  // wind direction and speed in global
  Eigen::Vector2d wind_global;  // direction, speed
};

#endif /*_WINDCOMPENSATION_H_*/

// 多项式插值得到风力