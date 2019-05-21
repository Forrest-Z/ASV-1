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

template <USEKALMAN indicator_kalman>
class estimator {
 public:
  explicit estimator(const vessel& _vessel, const estimatordata& _estimatordata)
      : roll_outlierremove(_vessel.roll_v(1), _vessel.roll_v(0),
                           _estimatordata.sample_time),
        surgev_outlierremove(_vessel.x_thrust(1) / _vessel.Mass(0, 0),
                             _vessel.x_thrust(0) / _vessel.Mass(0, 0),
                             _estimatordata.sample_time),
        swayv_outlierremove(_vessel.y_thrust(1) / _vessel.Mass(1, 1),
                            _vessel.y_thrust(0) / _vessel.Mass(1, 1),
                            _estimatordata.sample_time),
        _kalmanfilterv(_vessel, _estimatordata.sample_time),
        former_heading(0),
        sample_time(_estimatordata.sample_time) {}
  estimator() = delete;
  ~estimator() {}

  // setvalue after the initialization
  void setvalue(estimatorRTdata& _RTdata, double gps_x, double gps_y,
                double gps_z, double gps_roll, double gps_pitch,
                double gps_heading, double gps_Ve, double gps_Vn) {
    // heading rate
    former_heading = gps_heading * M_PI / 180;
    // outlier removal
    roll_outlierremove.setlastvalue(gps_roll);
    surgev_outlierremove.setlastvalue(0);
    swayv_outlierremove.setlastvalue(0);
    // low pass
    x_lowpass.setaveragevector(gps_x);
    y_lowpass.setaveragevector(gps_y);
    heading_lowpass.setaveragevector(former_heading);
    roll_lowpass.setaveragevector(gps_roll * M_PI / 180);

    // measurement, position_6dof
    preprocesssGPSdata(_RTdata, gps_x, gps_y, gps_z, gps_roll, gps_pitch,
                       gps_heading, gps_Ve, gps_Vn);
    _RTdata.State = _RTdata.Measurement;
    // Kalman filtering
    _kalmanfilterv.setState(_RTdata.State);
  }
  // update the estimated force acting on the vessel
  void updateestimatedforce(estimatorRTdata& _RTdata,
                            const Eigen::Vector3d& _thrust,
                            const Eigen::Vector3d& _wind) {
    _RTdata.BalphaU = _thrust + _wind;
  }
  // read sensor data and perform state estimation
  void estimatestate(estimatorRTdata& _RTdata, double gps_x, double gps_y,
                     double gps_z, double gps_roll, double gps_pitch,
                     double gps_heading, double gps_Ve, double gps_Vn,
                     double _desiredheading) {
    preprocesssGPSdata(_RTdata, gps_x, gps_y, gps_z, gps_roll, gps_pitch,
                       gps_heading, gps_Ve, gps_Vn);

    // calculate the coordinate transform matrix
    calculateCoordinateTransform(_RTdata.CTG2B, _RTdata.CTB2G,
                                 _RTdata.Measurement(2), _desiredheading);

    if constexpr (indicator_kalman == KALMANON)
      _RTdata.State =
          _kalmanfilterv.kalmanonestep(_RTdata).getState();  // kalman filtering
    else
      _RTdata.State = _RTdata.Measurement;  // use low-pass filtering only
  }
  // read sensor data and perform state estimation (simulation)
  void estimatestate(estimatorRTdata& _RTdata, double _desiredheading) {
    // calculate the coordinate transform matrix
    _RTdata.Measurement = _RTdata.State;
    calculateCoordinateTransform(_RTdata.CTG2B, _RTdata.CTB2G,
                                 _RTdata.Measurement(2), _desiredheading);
    _RTdata.State =
        _kalmanfilterv.kalmanonestep(_RTdata).getState();  // kalman filtering
  }
  // realtime calculation of position and velocity errors
  void estimateerror(estimatorRTdata& _RTdata,
                     const Eigen::Vector3d& _setpoints,
                     const Eigen::Vector3d& _vsetpoints) {
    Eigen::Vector3d _perror = Eigen::Vector3d::Zero();
    for (int i = 0; i != 2; ++i) _perror(i) = _setpoints(i) - _RTdata.State(i);
    _perror(2) = shortestheading(_setpoints(2) - _RTdata.State(2));
    _RTdata.p_error = _RTdata.CTG2B * _perror;
    _RTdata.v_error = _vsetpoints - _RTdata.State.tail(3);
  }

  double getsampletime() const noexcept { return sample_time; }

 private:
  // variable for low passing
  // lowpass<5> x_lowpass;
  // lowpass<5> y_lowpass;
  // lowpass<10> heading_lowpass;
  // lowpass<10> roll_lowpass;
  // lowpass<5> surgev_lowpass;
  // lowpass<5> swayv_lowpass;
  // lowpass<5> yawv_lowpass;

  lowpass<1> x_lowpass;
  lowpass<1> y_lowpass;
  lowpass<1> heading_lowpass;
  lowpass<1> roll_lowpass;
  lowpass<1> surgev_lowpass;
  lowpass<1> swayv_lowpass;
  lowpass<1> yawv_lowpass;
  // variable for outlier removal
  outlierremove roll_outlierremove;
  outlierremove surgev_outlierremove;
  outlierremove swayv_outlierremove;

  kalmanfilterv<> _kalmanfilterv;

  double former_heading;  // heading rate estimation
  double sample_time;

  // calculate the real time coordinate transform matrix
  void calculateCoordinateTransform(Eigen::Matrix3d& _CTG2B,
                                    Eigen::Matrix3d& _CTB2G, double _rtheading,
                                    double desired_heading) {
    double cvalue = 0.0;
    double svalue = 0.0;

    if (abs(shortestheading(_rtheading - desired_heading)) < M_PI / 36) {
      // use the fixed setpoint orientation to prevent measurement noise
      cvalue = std::cos(desired_heading);
      svalue = std::sin(desired_heading);
    } else {
      // if larger than 5 deg, we use the realtime orientation
      cvalue = std::cos(_rtheading);
      svalue = std::sin(_rtheading);
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
    double _gps_heading = restrictheadingangle(gps_heading * M_PI / 180);
    double _gps_Ve = surgev_outlierremove.removeoutlier(gps_Ve);
    double _gps_Vn = swayv_outlierremove.removeoutlier(gps_Vn);
    // update raw measured data from GPS/IMU sensors
    _RTdata.Measurement(0) = x_lowpass.movingaverage(_gps_x);
    _RTdata.Measurement(1) = y_lowpass.movingaverage(_gps_y);
    _RTdata.Measurement(2) = heading_lowpass.movingaverage(_gps_heading);
    _RTdata.Measurement(3) = surgev_lowpass.movingaverage(_gps_Ve);
    _RTdata.Measurement(4) = swayv_lowpass.movingaverage(_gps_Vn);
    _RTdata.Measurement(5) = yawv_lowpass.movingaverage(calheadingrate(
        _RTdata.Measurement(2)));  // we have to estimate the heading rate
    _RTdata.motiondata_6dof(0) = _RTdata.Measurement(0);
    _RTdata.motiondata_6dof(1) = _RTdata.Measurement(1);
    _RTdata.motiondata_6dof(2) = _gps_z;
    _RTdata.motiondata_6dof(3) = roll_lowpass.movingaverage(_gps_roll);
    _RTdata.motiondata_6dof(4) = _gps_pitch;
    _RTdata.motiondata_6dof(5) = _RTdata.Measurement(2);
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
  // restrict heading angle (0-2PI) to (-PI ~ PI)
  double restrictheadingangle(double _heading) noexcept {
    if (_heading > M_PI) _heading -= (2 * M_PI);
    return _heading;
  }
};

#endif /* _ESTIMATOR_H_ */
