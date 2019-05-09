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

#include "pidcontroller.h"
#include "thrustallocation.h"
#include "windcompensation.h"

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
      : sample_time(_controllerdata.sample_time),
        _controlmode(_controllerdata.controlmode),
        _windcompensation(_controllerdata.windcompensation),
        _windestimation({}),
        _pidcontroller(_piddata, _sample_time),
        _thrustallocation(_RTdata, _thrustallocationdata, _v_tunnelthrusterdata,
                          _v_azimuththrusterdata) {}

  // automatic control using pid controller and QP-based thrust allocation
  void pidcontrolleronestep(controllerRTdata<m, n> &_controllerdata,
                            const vectornd &_error, const vectornd &_derror,
                            const vectornd &_feedforward) {
    switch (_windcompensation) {
      case WINDOFF:
        vectornd windload = vectornd::Zero();
        break;
      case WINDON:
        break;
      default:
        break;
    }

    _controllerdata.tau =
        _pidcontroller.calculategeneralizeforce(_error, _derror);
    _thrustallocation.onestepthrusterallocation(_realtimedata);
  }

  // automatic heading control, and manual control in x, y direction
  void headingcontrolleronestep(realtimevessel_first &_realtimedata, int xforce,
                                int yforce) {
    mykalmanfilter.kalmanonestep(_realtimedata);
    mypidcontroller.calculategeneralizeforce(_realtimedata);
    setGeneralizeForce(_realtimedata, xforce, yforce);
    mythrusterallocation.onestepthrusterallocation(_realtimedata);
  }

  // manual control in x,y and Mz direction
  void fullymanualcontroller(int xforce, int yforce, int zmoment,
                             realtimevessel_first &_realtimedata,
                             FILE *t_file) {
    setGeneralizeForce(_realtimedata, xforce, yforce, zmoment);
    mythrusterallocation.onestepthrusterallocation(_realtimedata, t_file);
  }

  // TODO
  vectornd getwindcompensation() {}
  Eigen::Vector3d getwindload() const {
    return _windcompensation.getwindload();
  }
  WINDCOMPENSATION getwindstatus() const { return windstatus; }
  void setwindstatus(WINDCOMPENSATION _windstatus) { windstatus = _windstatus; }

  // underactuated

  ~controller() {}

 private:
  double sample_time;
  CONTROLMODE _controlmode;
  WINDCOMPENSATION windstatus;
  windcompensation _windcompensation;
  pidcontroller<L, n> _pidcontroller;
  thrustallocation<m, n> _thrustallocation;
};

#endif /*_CONTROLLER_H_*/
