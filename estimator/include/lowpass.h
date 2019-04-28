/*
***********************************************************************
* lowpass.h: data processing including outlier removal,
* low pass filtering, etc
* This header file can be read by C++ compilers
*
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _LOWPASS_H_
#define _LOWPASS_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include "vesseldata.h"

template <int numsurge, int numsway, int numyaw, int numsurgev, int numswayv,
          int numyawv, int numroll = 1>
class lowpass {
  using VectorASurge = Eigen::Matrix<double, numsurge, 1>;
  using VectorASway = Eigen::Matrix<double, numsway, 1>;
  using VectorAYaw = Eigen::Matrix<double, numyaw, 1>;
  using VectorASurgeV = Eigen::Matrix<double, numsurgev, 1>;
  using VectorASwayV = Eigen::Matrix<double, numswayv, 1>;
  using VectorAYawV = Eigen::Matrix<double, numyawv, 1>;
  using VectorARoll = Eigen::Matrix<double, numroll, 1>;

 public:
  explicit lowpass(const vessel &_vessel)
      : average_surge(VectorASurge::Zero()),
        average_sway(VectorASway::Zero()),
        average_yaw(VectorAYaw::Zero()),
        average_surgev(VectorASurgeV::Zero()),
        average_swayv(VectorASwayV::Zero()),
        average_yawv(VectorAYawV::Zero()),
        average_roll(VectorARoll::Zero()) {
    initializekalman(_vessel);
  }
  lowpass() = delete;
  ~lowpass() {}

 private:
  VectorASurge average_surge;
  VectorASway average_sway;
  VectorAYaw average_yaw;
  VectorASurgeV average_surgev;
  VectorASwayV average_swayv;
  VectorAYawV average_yawv;
  VectorARoll average_roll;

  double movingaverage_yaw(double _dtheta) {
    // pop_front
    VectorAYaw t_average_yaw = VectorAYaw::Zero();
    t_average_yaw.head(numyaw - 1) = average_yaw.tail(numyaw - 1);
    // push back
    t_average_yaw(numyaw - 1) = _dtheta;
    average_yaw = t_average_yaw;
    // calculate the mean value
    return average_yaw.mean();
  }
  double movingaverage_surge(double _dx) {
    // pop_front
    VectorASurge t_average_surge = VectorASurge::Zero();
    t_average_surge.head(numsurge - 1) = average_surge.tail(numsurge - 1);
    // push back
    t_average_surge(numsurge - 1) = _dx;
    average_surge = t_average_surge;
    // calculate the mean value
    return average_surge.mean();
  }
  double movingaverage_sway(double _dy) {
    // pop_front
    VectorASway t_average_sway = VectorASway::Zero();
    t_average_sway.head(numsway - 1) = average_sway.tail(numsway - 1);
    // push back
    t_average_sway(numsway - 1) = _dy;
    average_sway = t_average_sway;
    // calculate the mean value
    return average_sway.mean();
  }
  double movingaverage_roll(double _roll) {
    // pop_front
    VectorARoll t_average_roll = VectorARoll::Zero();
    t_average_roll.head(numroll - 1) = average_roll.tail(numroll - 1);
    // push back
    t_average_roll(numroll - 1) = _roll;
    average_roll = t_average_roll;
    // calculate the mean value
    return average_roll.mean();
  }
  double movingaverage_yaw_velocity(double _vtheta) {
    double movingaverage_roll(double _roll) {
      // pop_front
      VectorARoll t_average_roll = VectorARoll::Zero();
      t_average_roll.head(numroll - 1) = average_roll.tail(numroll - 1);
      // push back
      t_average_roll(numroll - 1) = _roll;
      average_roll = t_average_roll;
      // calculate the mean value
      return average_roll.mean();
    }
  }
  double movingaverage_surge_velocity(double _vx);
  double movingaverage_sway_velocity(double _vy);
};

#endif /* _DATAPROCESS_H_ */