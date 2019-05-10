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

template <int n = 3>
class windcompensation {
  using vectornd = Eigen::Matrix<double, n, 1>;

 public:
  windcompensation()
      : load(vectornd::Zero()),
        wind_body(Eigen::Vector2d::Zero()),
        wind_global(Eigen::Vector2d::Zero()) {}

  windcompensation& computewindload(const Eigen::Vector2d& _wind_body) {
    wind_body = _wind_body;

    load(0) = wind_body(0) * 0;
    load(1) = wind_body(0) * 0;
    load(2) = wind_body(0) * 0;
    return *this;
  }

  // TODO
  Eigen::Vector2d convertbwind2global(const Eigen::Vector2d& _windbody,
                                      double v_heading, double v_speed) {
    Eigen::Vector2d wind_global = Eigen::Vector2d::Zero();
    return wind_global;
  }

  // TODO
  vectornd getwindload() const { return load; }

 private:
  // Fx, Fy, Mz (wind force) in the body coordinate
  vectornd load;
  // wind direction and speed in body
  Eigen::Vector2d wind_body;  // direction, speed
  // wind direction and speed in global
  Eigen::Vector2d wind_global;  // direction, speed
};

#endif /*_WINDCOMPENSATION_H_*/

// 多项式插值得到风力, 风速和风向需要低通滤波