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
#include "vesseldata.h"

// n: # of dimension of control space
// m: # of all thrusters on the vessel
// L: # of integral length of PID controller
template <int L, int m, ACTUATION index_actuation, int n = 3>
class controller {
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixnld = Eigen::Matrix<double, n, L>;
  using matrixpid = Eigen::Matrix<double, 3, n>;

 public:
  // fully actuated
  explicit controller(
      const controllerdata &_controllerdata, const vessel &_vessel,
      const std::vector<pidcontrollerdata> &_piddata,
      const thrustallocationdata &_thrustallocationdata,
      const std::vector<tunnelthrusterdata> &_v_tunnelthrusterdata,
      const std::vector<azimuththrusterdata> &_v_azimuththrusterdata,
      const std::vector<ruddermaindata> &_v_ruddermaindata)
      : pids(matrixpid::Zero()),
        v_allowed_error(vectornd::Zero()),
        v_max_output(vectornd::Zero()),
        v_min_output(vectornd::Zero()),
        error_integralmatrix(matrixnld::Zero()),
        damping(_vessel.Damping),
        sample_time(_controllerdata.sample_time),
        controlmode(_controllerdata.controlmode),
        windstatus(_controllerdata.windstatus),
        _thrustallocation(_thrustallocationdata, _v_tunnelthrusterdata,
                          _v_azimuththrusterdata, _v_ruddermaindata) {
    setPIDmatrix(_piddata);
    initializepidcontroller(_piddata);
  }
  controller() = delete;
  ~controller() {}

  void initializecontroller(controllerRTdata<m, n> &_RTdata) {
    _thrustallocation.initializapropeller(_RTdata);
  }

  void setcontrolmode(CONTROLMODE _controlmode) {
    controlmode = _controlmode;
    _thrustallocation.setQ(_controlmode);
  }

  // automatic control using pid controller and QP-based thrust allocation
  void controlleronestep(controllerRTdata<m, n> &_controllerdata,
                         const vectornd &_windload, const vectornd &_error,
                         const vectornd &_derror, const vectornd &_command,
                         double _desired_u = 0) {
    // PID controller
    vectornd d_tau = vectornd::Zero();
    vectornd position_error_integral = updateIntegralMatrix(_error);
    if (!compareerror(_error)) {
      for (int i = 0; i != n; ++i)
        d_tau(i) = pids(0, i) * _error(i)  // proportional term
                   + pids(1, i) * position_error_integral(i)  // integral term
                   + pids(2, i) * _derror(i);                 // derivative term
    }

    // Drag in the x direction
    d_tau(0) += damping(0, 0) * _desired_u;

    switch (windstatus) {
      case WINDOFF:
        break;
      case WINDON:  // TODO
        d_tau += _windload;
        break;
      default:
        break;
    }

    // controller mode
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
    _controllerdata.tau = d_tau;

    // thrust allocation
    _thrustallocation.onestepthrustallocation(_controllerdata);
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

  WINDCOMPENSATION getwindstatus() const { return windstatus; }
  void setwindstatus(WINDCOMPENSATION _windstatus) { windstatus = _windstatus; }
  double getsampletime() const noexcept { return sample_time; }

 private:
  matrixpid pids;                  // pid matrix
  vectornd v_allowed_error;        // allowed_error
  vectornd v_max_output;           // max_output
  vectornd v_min_output;           // min_output
  matrixnld error_integralmatrix;  // I
  Eigen::Matrix3d damping;         //
  double sample_time;
  CONTROLMODE controlmode;
  WINDCOMPENSATION windstatus;

  thrustallocation<m, index_actuation, n> _thrustallocation;

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
    // t_integralmatrix.col(index) = sample_time * _error;
    t_integralmatrix.col(index) = _error;
    error_integralmatrix = t_integralmatrix;
    return error_integralmatrix.rowwise().mean();
  }
  // compare the real time error with the allowed error
  bool compareerror(const vectornd &_error) {
    for (int i = 0; i != n; ++i)
      if (std::abs(_error(i)) > v_allowed_error(i)) return false;
    return true;
  }
};

#endif /*_CONTROLLER_H_*/
