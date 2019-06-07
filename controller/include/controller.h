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
  using matrixpid = Eigen::Matrix<double, 2, n>;

 public:
  // fully actuated
  explicit controller(
      const controllerdata &_controllerdata, const vessel &_vessel,
      const std::vector<pidcontrollerdata> &_piddata,
      const thrustallocationdata &_thrustallocationdata,
      const std::vector<tunnelthrusterdata> &_v_tunnelthrusterdata,
      const std::vector<azimuththrusterdata> &_v_azimuththrusterdata,
      const std::vector<ruddermaindata> &_v_ruddermaindata)
      : position_pids(matrixpid::Zero()),
        velocity_pids(matrixpid::Zero()),
        position_allowed_error(vectornd::Zero()),
        velocity_allowed_error(vectornd::Zero()),
        v_max_output(vectornd::Zero()),
        v_min_output(vectornd::Zero()),
        positionerror_integralmatrix(matrixnld::Zero()),
        velocityerror_integralmatrix(matrixnld::Zero()),
        damping(_vessel.Damping),
        sample_time(_controllerdata.sample_time),
        controlmode(_controllerdata.controlmode),
        windstatus(_controllerdata.windstatus),
        _thrustallocation(_thrustallocationdata, _v_tunnelthrusterdata,
                          _v_azimuththrusterdata, _v_ruddermaindata) {
    setuppidcontroller(_piddata);
  }
  controller() = delete;
  ~controller() {}

  void initializecontroller(controllerRTdata<m, n> &_RTdata) {
    _thrustallocation.initializapropeller(_RTdata);
  }

  // automatic control using pid controller and QP-based thrust allocation
  void controlleronestep(controllerRTdata<m, n> &_controllerdata,
                         const vectornd &_windload, const vectornd &_error,
                         const vectornd &_derror, const vectornd &_command,
                         const vectornd &_desired_speed) {
    vectornd d_tau = vectornd::Zero();

    // position control
    positionloop(d_tau, _error);

    // velocity control
    velocityloop(d_tau, _derror);

    // linear damping compensation
    compensatelineardamping(d_tau, _desired_speed);

    // wind compensation (TODO)
    windcompensation(d_tau, _windload);

    commandfromjoystick(d_tau, _command);

    // restrict desired force
    restrictdesiredforce(d_tau);

    // std::cout << d_tau << std::endl;
    // thrust allocation
    _controllerdata.tau = d_tau;
    _thrustallocation.onestepthrustallocation(_controllerdata);
  }

  // assign value to pid controller (from GUI)
  void setPIDmatrix(
      const std::vector<pidcontrollerdata> &_v_pidcontrollerdata) {
    for (int i = 0; i != n; ++i) {
      position_pids(0, i) = _v_pidcontrollerdata[i].position_P;
      position_pids(1, i) = _v_pidcontrollerdata[i].position_I;
      velocity_pids(0, i) = _v_pidcontrollerdata[i].velocity_P;
      velocity_pids(1, i) = _v_pidcontrollerdata[i].velocity_I;
    }
  }

  void setcontrolmode(CONTROLMODE _controlmode) {
    controlmode = _controlmode;
    _thrustallocation.setQ(_controlmode);
  }

  void setcontrolmode(int _controlmode) {
    switch (_controlmode) {  // controller mode
      case 0:
        controlmode = MANUAL;
        break;
      case 1:
        controlmode = HEADINGONLY;
        break;
      case 2:
        controlmode = MANEUVERING;
        break;
      case 3:
        controlmode = DYNAMICPOSITION;
        break;
      default:
        break;
    }
    setcontrolmode(controlmode);
  }
  WINDCOMPENSATION getwindstatus() const { return windstatus; }
  void setwindstatus(int _windstatus) {
    switch (_windstatus) {
      case 0:
        windstatus = WINDOFF;
        break;
      case 1:
        windstatus = WINDON;
        break;
      default:
        break;
    }
  }
  void setwindstatus(WINDCOMPENSATION _windstatus) { windstatus = _windstatus; }
  double getsampletime() const noexcept { return sample_time; }

 private:
  matrixpid position_pids;  // pid matrix for position control
  matrixpid velocity_pids;  // pid matrix for velocity control

  vectornd position_allowed_error;  // allowed_error for position control
  vectornd velocity_allowed_error;  // allowed_error for position control

  vectornd v_max_output;  // max_output of thruster
  vectornd v_min_output;  // min_output of thruster

  matrixnld positionerror_integralmatrix;  // I for position control
  matrixnld velocityerror_integralmatrix;  // I for velocity control

  Eigen::Matrix3d damping;  //
  double sample_time;
  CONTROLMODE controlmode;
  WINDCOMPENSATION windstatus;

  thrustallocation<m, index_actuation, n> _thrustallocation;

  void setuppidcontroller(
      const std::vector<pidcontrollerdata> &_vcontrollerdata) {
    for (int i = 0; i != n; ++i) {
      position_allowed_error(i) = _vcontrollerdata[i].position_allowed_error;
      velocity_allowed_error(i) = _vcontrollerdata[i].velocity_allowed_error;
      v_max_output(i) = _vcontrollerdata[i].max_output;
      v_min_output(i) = _vcontrollerdata[i].min_output;
    }
    setPIDmatrix(_vcontrollerdata);
  }

  void positionloop(vectornd &_tau, const vectornd &_error) {
    // position control
    vectornd position_error_integral = updatepositionIntegralMatrix(_error);
    switch (controlmode) {  // controller mode
      case MANUAL:
        break;
      case MANEUVERING:
        _tau(2) += position_pids(0, 2) * _error(2)  // proportional term
                   + position_pids(1, 2) *
                         position_error_integral(2);  // integral term
        break;
      case DYNAMICPOSITION:
      case HEADINGONLY:
        if (!comparepositionerror(_error)) {
          for (int i = 0; i != n; ++i)
            _tau(i) += position_pids(0, i) * _error(i)  // proportional term
                       + position_pids(1, i) *
                             position_error_integral(i);  // integral term
        }
        break;
      default:
        break;
    }

  }  // positionloop()

  void velocityloop(vectornd &_tau, const vectornd &_derror) {
    // velocity control
    vectornd velocity_error_integral = updatevelocityIntegralMatrix(_derror);

    switch (controlmode) {  // controller mode
      case MANUAL:
        break;
      case MANEUVERING:
        _tau(0) += velocity_pids(0, 0) * _derror(0)  // proportional term
                   + velocity_pids(1, 0) *
                         velocity_error_integral(0);  // integral term
        _tau(2) += velocity_pids(0, 2) * _derror(2)   // proportional term
                   + velocity_pids(1, 2) *
                         velocity_error_integral(2);  // integral term
        break;
      case DYNAMICPOSITION:
      case HEADINGONLY:
        if (!comparevelocityerror(_derror)) {
          for (int i = 0; i != n; ++i)
            _tau(i) += velocity_pids(0, i) * _derror(i)  // proportional term
                       + velocity_pids(1, i) *
                             velocity_error_integral(i);  // integral term
        }
        break;
      default:
        break;
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
  vectornd updatepositionIntegralMatrix(const vectornd &_error) {
    matrixnld t_integralmatrix = matrixnld::Zero();
    int index = L - 1;
    t_integralmatrix.leftCols(index) =
        positionerror_integralmatrix.rightCols(index);
    // t_integralmatrix.col(index) = sample_time * _error;
    t_integralmatrix.col(index) = _error;
    positionerror_integralmatrix = t_integralmatrix;
    return positionerror_integralmatrix.rowwise().mean();
  }

  // calculate the Integral error with moving window
  vectornd updatevelocityIntegralMatrix(const vectornd &_derror) {
    matrixnld t_integralmatrix = matrixnld::Zero();
    int index = L - 1;
    t_integralmatrix.leftCols(index) =
        velocityerror_integralmatrix.rightCols(index);
    t_integralmatrix.col(index) = _derror;
    velocityerror_integralmatrix = t_integralmatrix;
    return velocityerror_integralmatrix.rowwise().mean();
  }

  // compare the real time position error with the allowed error
  bool comparepositionerror(const vectornd &_error) {
    for (int i = 0; i != n; ++i)
      if (std::abs(_error(i)) > position_allowed_error(i)) return false;
    return true;
  }
  // compare the real time velocity error with the allowed error
  bool comparevelocityerror(const vectornd &_derror) {
    for (int i = 0; i != n; ++i)
      if (std::abs(_derror(i)) > velocity_allowed_error(i)) return false;
    return true;
  }
  // command from joystick
  void commandfromjoystick(vectornd &_tau, const vectornd &_command) {
    switch (controlmode) {  // controller mode
      case MANUAL:
        _tau = _command;
        break;
      case HEADINGONLY:
        _tau.head(n - 1) = _command.head(n - 1);
        break;
      case MANEUVERING:
        break;
      case DYNAMICPOSITION:
        break;
      default:
        break;
    }
  }

  // wind compensation
  void windcompensation(vectornd &_tau, const vectornd &_windload) {
    switch (windstatus) {
      case WINDOFF:
        break;
      case WINDON:  // TODO, negative or positive
        _tau -= _windload;
        break;
      default:
        break;
    }
  }

  // linear damping
  void compensatelineardamping(vectornd &_tau, const vectornd &_desired_speed) {
    for (int i = 0; i != n; ++i) _tau(i) += damping(i, i) * _desired_speed(i);
  }
};

#endif /*_CONTROLLER_H_*/
