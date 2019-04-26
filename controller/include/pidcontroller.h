/*
***********************************************************************
* pidcontroller.h:
* function for pid controller
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

#include <cstdlib>
#include <iostream>
#include <vector>
#include "controllerdata.h"

// n: # of dimension of control space
template <int n = 3>
class pidcontroller {
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixpid = Eigen::Matrix<double, 3, n>;

 public:
  explicit pidcontroller(const vessel_first &_vessel,
                         const std::vector<pidcontrollerdata> &_vcontrollerdata)
      : pids(matrixpid::Zero()), v_pidcontrollerdata(_vcontrollerdata) {
    setPIDmatrix(_vcontrollerdata);
  }
  pidcontroller() = delete;
  ~pidcontroller() {}
  // calculate the desired force using PID controller
  void calculategeneralizeforce(realtimevessel_first &_realtimedata) {
    // convert setpoint from global to body coordinate
    setsetpoints(_realtimedata);
    // calculate error
    position_error = setpoints_body - _realtimedata.State4control.head(3);
    if (compareerror(position_error)) {
      _realtimedata.tau.setZero();
    } else {  // proportional term
      Eigen::Vector3d Pout = matrix_P * position_error;
      // integral term
      updateIntegralMatrix(position_error);
      Eigen::Vector3d Iout = matrix_I * position_error_integral;
      // derivative term
      Eigen::Vector3d Dout = -matrix_D * _realtimedata.State4control.tail(3);
      // output
      _realtimedata.tau = Pout + Iout + Dout;
      // restrict desired force
      restrictdesiredforce(_realtimedata.tau);
    }
  }

  // assign value to pid controller
  void setPIDmatrix(const std::vector<pidcontrollerdata> &v_pidcontrollerdata) {
    for (int i = 0; i != n; ++i) {
      pids(0, i) = v_pidcontrollerdata[i].P;
      pids(1, i) = v_pidcontrollerdata[i].I;
      pids(2, i) = v_pidcontrollerdata[i].D;
    }
  }

 private:
  matrixpid pids;
  std::vector<pidcontrollerdata> v_pidcontrollerdata;

  void initializepidcontroller() {}
  // restrict the desired force to some value
  double restrictdesiredforce(double _input, double _min, double _max) {
    double t_input = _input;
    if (_input > _max) t_input = _max;
    if (_input < _min) t_input = _min;
    return t_input;
  }

  void restrictdesiredforce(vectornd &_desiredforce) {
    for (int i = 0; i != n; ++i)
      _desiredforce(i) = restrictdesiredforce(
          _desiredforce(i), v_pidcontrollerdata[i].min_output,
          v_pidcontrollerdata[i].max_output);
  }
};

#endif /*_PIDCONTROLLER_H_*/