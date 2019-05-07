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
  explicit estimator(const vessel& _vessel, double _sample_time, bool _kalman)
      : roll_outlierremove(_vessel.roll_v(1), _vessel.roll_v(0), _sample_time),
        surgev_outlierremove(_vessel.x_thrust(1) / _vessel.Mass(0, 0),
                             _vessel.x_thrust(0) / _vessel.Mass(0, 0),
                             _sample_time),
        swayv_outlierremove(_vessel.y_thrust(1) / _vessel.Mass(1, 1),
                            _vessel.y_thrust(0) / _vessel.Mass(1, 1),
                            _sample_time),
        _kalmanfilterv(_vessel, _sample_time),
        former_heading(0),
        sample_time(_sample_time),
        kalman_use(_kalman) {}
  ~estimator() {}

  void setvalue(estimatorRTdata& _RTdata) {
    //
    preprocesssensordata(_RTdata);

    State << _x, _y, _heading, 0, 0, 0;
    _kalmanfilterv.setState(State);
    roll_outlierremove.setlastvalue(_roll);
    former_heading = ;
    // assume that the initial velocities are all zero
    surgev_outlierremove.setlastvalue(0);
    swayv_outlierremove.setlastvalue(0);
    yawv_outlierremove.setlastvalue(0);
  }
  // read sensor data and perform state estimation
  void estimatestate(estimatorRTdata& _RTdata, double gps_x, double gps_y,
                     double gps_z, double gps_roll, double gps_pitch,
                     double gps_heading, double gps_Ve, double gps_Vn) {
    preprocesssGPSdata(_RTdata, gps_x, gps_y, gps_z, gps_roll, gps_pitch,
                       gps_heading, gps_Ve, gps_Vn);
    if (kalman_use)
      _RTdata.State =
          _kalmanfilterv.kalmanonestep(_RTdata).getState();  // kalman filtering
    else
      _RTdata.State = _RTdata.measuremment;  // use low-pass filtering only
  }

  void estimateerror(estimatorRTdata& _RTdata,
                     const Eigen::Matrix<double, 6, 1>& _setpoints) {}

 private:
  // variable for low passing
  lowpass<5> x_lowpass;
  lowpass<5> y_lowpass;
  lowpass<10> heading_lowpass;
  lowpass<10> roll_lowpass;
  lowpass<5> surgev_lowpass;
  lowpass<5> swayv_lowpass;
  lowpass<5> yawv_lowpass;
  // variable for outlier removal
  outlierremove roll_outlierremove;
  outlierremove surgev_outlierremove;
  outlierremove swayv_outlierremove;

  kalmanfilterv _kalmanfilterv;

  double former_heading;  // heading rate estimation
  double sample_time;

  bool kalman_use;  // use kalman filtering or not:
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

  // outlier removal, unit converstion, low pass filtering
  void preprocesssGPSdata(estimatorRTdata& _RTdata, double gps_x, double gps_y,
                          double gps_z, double gps_roll, double gps_pitch,
                          double gps_heading, double gps_Ve, double gps_Vn) {
    // change direction, convert rad to deg, outlier removal or ....
    double _gps_x = gps_x;
    double _gps_y = gps_y;
    double _gps_z = -gps_z;
    double _gps_roll = roll_outlierremove.removeoutlier(gps_roll * M_PI / 180);
    double _gps_pitch = gps_pitch * M_PI / 180;
    double _gps_heading = gps_heading * M_PI / 180;
    double _gps_Ve = surgev_outlierremove.removeoutlier(gps_Ve);
    double _gps_Vn = swayv_outlierremove.removeoutlier(gps_Vn);
    // update raw measured data from GPS/IMU sensors
    _RTdata.measuremment(0) = x_lowpass.movingaverage(_gps_x);
    _RTdata.measuremment(1) = y_lowpass.movingaverage(_gps_y);
    _RTdata.measuremment(2) = heading_lowpass.movingaverage(_gps_heading);
    _RTdata.measuremment(3) = surgev_lowpass.movingaverage(_gps_Ve);
    _RTdata.measuremment(4) = swayv_lowpass.movingaverage(_gps_Vn);
    _RTdata.measuremment(5) = yawv_lowpass.movingaverage(calheadingrate(
        _RTdata.measuremment(2)));  // we have to estimate the heading rate
    _RTdata.motiondata_6dof(0) = _RTdata.measuremment(0);
    _RTdata.motiondata_6dof(1) = _RTdata.measuremment(1);
    _RTdata.motiondata_6dof(2) = _gps_z;
    _RTdata.motiondata_6dof(3) = roll_lowpass.movingaverage(_gps_roll);
    _RTdata.motiondata_6dof(4) = _gps_pitch;
    _RTdata.motiondata_6dof(5) = _RTdata.measuremment(2);
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
  // calculate the heading rate
  double calheadingrate(double _newvalue) {
    double delta_yaw = shortestheading(_newvalue - former_heading);
    former_heading = _newvalue;
    return delta_yaw / sample_time;
  }
};

#endif /* _ESTIMATOR_H_ */
