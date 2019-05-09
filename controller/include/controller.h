/*
***********************************************************************
* controller.h:
* function for controller for fully-actuated USV or
* underactuated USV (including pid controller, thrust allocation,etc)
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <cstdlib>
#include <iostream>
#include <vector>
#include "controllerdata.h"
#include "thrustallocation.h"
#include "windcompensation.h"

// n: # of dimension of control space
// m: # of all thrusters on the vessel
// L: # of integral length of PID controller
template <int L, int m, int n = 3>
class controller {
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixnld = Eigen::Matrix<double, n, L>;
  using matrixpid = Eigen::Matrix<double, 3, n>;

 public:
  // fully actuated
  explicit controller(
      controllerRTdata<m, n> &_RTdata, const controllerdata &_controllerdata,
      const std::vector<pidcontrollerdata> &_piddata,
      const thrustallocationdata &_thrustallocationdata,
      const std::vector<tunnelthrusterdata> &_v_tunnelthrusterdata,
      const std::vector<azimuththrusterdata> &_v_azimuththrusterdata)
      : pids(matrixpid::Zero()),
        v_allowed_error(vectornd::Zero()),
        v_max_output(vectornd::Zero()),
        v_min_output(vectornd::Zero()),
        error_integralmatrix(matrixnld::Zero()),
        sample_time(_controllerdata.sample_time),
        controlmode(_controllerdata.controlmode),
        windstatus(_controllerdata.windstatus),
        windload(vectornd::Zero()),
        d_tau(vectornd::Zero()),
        _thrustallocation(_RTdata, _thrustallocationdata, _v_tunnelthrusterdata,
                          _v_azimuththrusterdata) {
    setPIDmatrix(_piddata);
    initializepidcontroller(_piddata);
  }

  ~controller() {}

  void setcontrolmode(CONTROLMODE _controlmode) {
    controlmode = _controlmode;
    _thrustallocation.setQ(_controlmode);
  }

  // automatic control using pid controller and QP-based thrust allocation
  void controlleronestep(controllerRTdata<m, n> &_controllerdata,
                         const vectornd &_error, const vectornd &_derror,
                         windestimation<n> &_wind, const vecornd &_command) {
    _wind.load =
        _windcompensation.computewindload(_wind.wind_body).getwindload();
    switch (windstatus) {
      case WINDOFF:
        windload.setZero();
        break;
      case WINDON:  // TODO
        windload = _wind.load;
        break;
      default:
        break;
    }
    calculategeneralizeforce(_error, _derror, windload, _command);
    _controllerdata.tau = d_tau;
    _thrustallocation.onestepthrusterallocation(_controllerdata);
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

  // TODO

  WINDCOMPENSATION getwindstatus() const { return windstatus; }
  void setwindstatus(WINDCOMPENSATION _windstatus) { windstatus = _windstatus; }

  // underactuated

 private:
  matrixpid pids;                  // pid matrix
  vectornd v_allowed_error;        // allowed_error
  vectornd v_max_output;           // max_output
  vectornd v_min_output;           // min_output
  matrixnld error_integralmatrix;  // I

  double sample_time;
  CONTROLMODE controlmode;
  WINDCOMPENSATION windstatus;
  vectornd windload;
  vectornd d_tau;  // prevent "bump" in the desired force
  windcompensation<n> _windcompensation;
  thrustallocation<m, n> _thrustallocation;

  // calculate the desired force using PID controller
  void calculategeneralizeforce(const vectornd &_error, const vectornd &_derror,
                                const vectornd &_feedforward,
                                const vectornd &_command) {
    d_tau.setZero();
    vectornd position_error_integral = updateIntegralMatrix(_error);
    if (!compareerror(_error)) {
      for (int i = 0; i != n; ++i)
        d_tau(i) = pids(0, i) * _error(i)  // proportional term
                   + pids(1, i) * position_error_integral(i)  // integral term
                   + pids(2, i) * _derror(i);                 // derivative term
    }
    // add the wind compensation
    d_tau += _feedforward;

    switch (controlmode) {
      case MANUAL:
        d_tau = _command;
        break;
      case HEADINGONLY:
        for (int i = 0; i != 2; ++i) d_tau(i) = _command(i);
        break;
      case AUTOMATIC:
        break;
      default:
        break;
    }

    // restrict desired force
    restrictdesiredforce(d_tau);
  }

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

#endif /*_CONTROLLER_H_*/
