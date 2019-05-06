/*
***********************************************************************
* estimator.h: state estimation of USV
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/
#ifndef _ESTIMATOR_H_
#define _ESTIMATOR_H_

#include "kalmanfilter.h"
#include "lowpass.h"
#include "outlierremove.h"

class estimator {
 public:
  explicit estimator(const vessel& _vessel, double sample_time),
      surge_outlierremove(_vessel.surge_v(1), _vessel.surge_v(0), sample_time),
      sway_outlierremove(_vessel.sway_v(1), _vessel.sway_v(0), sample_time),
      yaw_outlierremove(_vessel.yaw_v(1), _vessel.yaw_v(0), sample_time),
      roll_outlierremove(_vessel.roll_v(1), _vessel.roll_v(0), sample_time),
      surgev_outlierremove(_vessel.x_thrust(1) / _vessel.Mass(0, 0),
                           _vessel.x_thrust(0) / _vessel.Mass(0, 0),
                           sample_time),
      swayv_outlierremove(_vessel.sway_v(1), _vessel.sway_v(0), sample_time), {}
  ~estimator() {}

  void estimatoronestep(estimatorRTdata& _RTdata) {
    preprocesssensordata(_RTdata);
    // outliour remove
    double t_surge = surge_outlierremove.removeoutlier(_RTdata.measuremment(0));
    double t_sway = sway_outlierremove.removeoutlier(_RTdata.measuremment(1));

    // low pass filtering
    surge_lowpass.movingaverage();
  }

 private:
  // variable for low passing
  lowpass<5> surge_lowpass;
  lowpass<5> sway_lowpass;
  lowpass<10> yaw_lowpass;
  lowpass<10> roll_lowpass;
  lowpass<5> surgev_lowpass;
  lowpass<5> swayv_lowpass;
  lowpass<5> yawv_lowpass;
  // variable for outlier removal
  outlierremove surge_outlierremove;
  outlierremove sway_outlierremove;
  outlierremove yaw_outlierremove;
  outlierremove roll_outlierremove;
  outlierremove surgev_outlierremove;
  outlierremove swayv_outlierremove;
  outlierremove yawv_outlierremove;

  kalmanfilterv<> _kalmanfilterv;
  // calculate the real time coordinate transform matrix
  void calculateCoordinateTransform(Eigen::Matrix3d& _CTG2B,
                                    Eigen::Matrix3d& _CTB2G,
                                    double realtime_orientation,
                                    double desired_orientation) {
    double cvalue = 0.0;
    double svalue = 0.0;
    if (abs(realtime_orientation - desired_orientation) < M_PI / 36) {
      // use the fixed setpoint orientation to prevent measurement noise
      cvalue = std::cos(desired_orientation);
      svalue = std::sin(desired_orientation);
    } else {
      // if larger than 5 deg, we use the realtime orientation
      cvalue = std::cos(realtime_orientation);
      svalue = std::sin(realtime_orientation);
    }

    _CTG2B(0, 0) = cvalue;
    _CTG2B(1, 1) = cvalue;
    _CTG2B(0, 1) = svalue;
    _CTG2B(1, 0) = -svalue;
    _CTB2G(0, 0) = cvalue;
    _CTB2G(1, 1) = cvalue;
    _CTB2G(0, 1) = -svalue;
    _CTB2G(1, 0) = svalue;
  }

  // change direction, convert rad to deg or ....
  void preprocesssensordata(estimatorRTdata& _RTdata) {
    _RTdata.measuremment(0) *= -1;  // surge, change the direction

    _RTdata.motiondata_6dof(3) *= (M_PI / 180);  // roll: deg -> rad
    _RTdata.motiondata_6dof(4) *= (M_PI / 180);  // pitch: deg -> rad
    _RTdata.measuremment(2) *= (M_PI / 180);     // yaw: deg -> rad
    _RTdata.motiondata_6dof(5) *= (M_PI / 180);  // yaw: deg -> rad
  }
  // find the shortest way to rotate
  double shortestheading(double _deltaheading) {
    double deltaheading = 0;
    if (_deltaheading > M_PI)
      deltaheading = _deltaheading - 2 * M_PI;
    else if (_deltaheading < -M_PI)
      deltaheading = _deltaheading + 2 * M_PI;
    else
      deltaheading = _deltaheading;
    return deltaheading;
  }
};

#endif /* _ESTIMATOR_H_ */
