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
template <int L, int n = 3>
class pidcontroller {
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixnld = Eigen::Matrix<double, n, L>;
  using matrixpid = Eigen::Matrix<double, 3, n>;

 public:
  explicit pidcontroller(const std::vector<pidcontrollerdata> &_vcontrollerdata,
                         double _sample_time)
      : pids(matrixpid::Zero()),
        v_allowed_error(vectornd::Zero()),
        v_max_output(vectornd::Zero()),
        v_min_output(vectornd::Zero()),
        error_integralmatrix(matrixnld::Zero()),
        sample_time(_sample_time) {
    setPIDmatrix(_vcontrollerdata);
    initializepidcontroller(_vcontrollerdata);
  }
  pidcontroller() = delete;
  ~pidcontroller() {}
  // calculate the desired force using PID controller
  vectornd calculategeneralizeforce(const vectornd &_error,
                                    const vectornd &_derror,
                                    const vectornd &_feedforward) {
    vectornd _desiredforce = vectornd::Zero();
    vectornd position_error_integral = updateIntegralMatrix(_error);
    if (!compareerror(_error)) {
      for (int i = 0; i != n; ++i)
        _desiredforce(i) =
            pids(0, i) * _error(i)                     // proportional term
            + pids(1, i) * position_error_integral(i)  // integral term
            - pids(2, i) * _derror(i);                 // derivative term
    }
    // add the wind compensation
    _desiredforce += _feedforward;
    // restrict desired force
    restrictdesiredforce(_desiredforce);
    return _desiredforce;
  }

  // assign value to pid controller
  void setPIDmatrix(
      const std::vector<pidcontrollerdata> &_v_pidcontrollerdata) {
    for (int i = 0; i != n; ++i) {
      pids(0, i) = _v_pidcontrollerdata[i].P;
      pids(1, i) = _v_pidcontrollerdata[i].I;
      pids(2, i) = _v_pidcontrollerdata[i].D;
    }
  }

 private:
  matrixpid pids;                  // pid matrix
  vectornd v_allowed_error;        // allowed_error
  vectornd v_max_output;           // max_output
  vectornd v_min_output;           // min_output
  matrixnld error_integralmatrix;  // I
  double sample_time;              //

  void initializepidcontroller(
      const std::vector<pidcontrollerdata> &_vcontrollerdata) {
    for (int i = 0; i != n; ++i) {
      v_allowed_error(i) = _vcontrollerdata[i].allowed_error;
      v_max_output(i) = _vcontrollerdata[i].max_output;
      v_min_output(i) = _vcontrollerdata[i].min_output;
    }
  }
  // restrict the desired force to some value
  double restrictdesiredforce(double _input, double _min, double _max) {
    double t_input = _input;
    if (_input > _max) t_input = _max;
    if (_input < _min) t_input = _min;
    return t_input;
  }

  void restrictdesiredforce(vectornd &_desiredforce) {
    for (int i = 0; i != n; ++i)
      _desiredforce(i) = restrictdesiredforce(_desiredforce(i), v_min_output(i),
                                              v_max_output(i));
  }
  // calculate the Integral error with moving window
  vectornd updateIntegralMatrix(const vectornd &_error) {
    matrixnld t_integralmatrix = matrixnld::Zero();
    int index = L - 1;
    t_integralmatrix.leftCols(index) = error_integralmatrix.rightCols(index);
    t_integralmatrix.col(index) = sample_time * _error;
    error_integralmatrix = t_integralmatrix;
    return error_integralmatrix.rowwise().sum();
  }
  // compare the real time error with the allowed error
  bool compareerror(const vectornd &_error) {
    for (int i = 0; i != n; ++i)
      if (std::abs(_error(i)) > v_allowed_error(i)) return false;
    return true;
  }
};

#endif /*_PIDCONTROLLER_H_*/