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

#include "estimatordata.h"
#include "lowpass.h"

class windcompensation {
 public:
  windcompensation()
      : load(Eigen::Vector3d::Zero()),
        wind_body(Eigen::Vector2d::Zero()),
        wind_global(Eigen::Vector2d::Zero()) {}

  windcompensation& computewindload(double wind_speed,
                                    double wind_orientation) {
    preprocessswinddata(wind_speed, wind_orientation);

    load(0) = wind_body(0) * 0;
    load(1) = wind_body(0) * 0;
    load(2) = wind_body(0) * 0;
    return *this;
  }

  // setvalue after the initialization
  void setvalue(double wind_speed, double wind_orientation) {
    // low pass
    speed_lowpass.setaveragevector(wind_speed);
    orientation_lowpass.setaveragevector(wind_orientation);
    preprocessswinddata(wind_speed, wind_orientation);
  }

  // TODO
  Eigen::Vector2d convertbwind2global(const Eigen::Vector2d& _windbody,
                                      double v_heading, double v_speed) {
    v_heading = 0;
    v_speed = 0;
    Eigen::Vector2d wind_global = Eigen::Vector2d::Zero();
    wind_global = _windbody;
    return wind_global;
  }

  Eigen::Vector3d getwindload() const { return load; }

 private:
  lowpass<5> speed_lowpass;
  lowpass<5> orientation_lowpass;
  // Fx, Fy, Mz (wind force) in the body coordinate
  Eigen::Vector3d load;
  // wind direction and speed in body
  Eigen::Vector2d wind_body;  // direction, speed
  // wind direction and speed in global
  Eigen::Vector2d wind_global;  // direction, speed

  void preprocessswinddata(double wind_speed, double wind_orientation) {
    wind_body(0) = orientation_lowpass.movingaverage(wind_orientation);
    wind_body(1) = speed_lowpass.movingaverage(wind_speed);
  }
};

#endif /*_WINDCOMPENSATION_H_*/

// TODO: 多项式插值得到风力, 风速和风向需要低通滤波