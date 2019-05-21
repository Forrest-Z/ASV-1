/*
***********************************************************************
* planner.h:
* function for motion planning
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _PLANNER_H_
#define _PLANNER_H_

#include "lineofsight.h"
#include "plannerdata.h"

class planner {
 public:
  explicit planner(const plannerdata &_plannerdata)
      : sample_time(_plannerdata.sample_time),
        _lineofsight(_plannerdata.los_radius, _plannerdata.los_capture_radius) {
  }
  planner() = delete;
  ~planner() {}

  bool switchwaypoint(plannerRTdata &_plannerRTdata,
                      const Eigen::Vector2d &_vesselposition,
                      const Eigen::Vector2d &newwaypoint) {
    if (_lineofsight.judgewaypoint(_vesselposition, _plannerRTdata.waypoint1)) {
      _plannerRTdata.waypoint0 = _plannerRTdata.waypoint1;
      _plannerRTdata.waypoint1 = newwaypoint;
      return true;
    }
    return false;
  }

  // path following using LOS
  planner &pathfollowLOS(plannerRTdata &_RTdata, const Eigen::Vector2d &_vp) {
    _RTdata.setpoint(2) =
        _lineofsight.computelospoint(_vp, _RTdata.waypoint0, _RTdata.waypoint1)
            .getdesired_theta();
    return *this;
  }
  planner &setconstantspeed(plannerRTdata &_plannerRTdata,
                            double _forwardspeed) {
    _plannerRTdata.v_setpoint(0) = _forwardspeed;
    return *this;
  }

  void setcommandfromjoystick(plannerRTdata &_plannerRTdata, double tau_x,
                              double tau_y, double tau_theta) {
    _plannerRTdata.command << tau_x, tau_y, tau_theta;
  }
  double getsampletime() const noexcept { return sample_time; }

 private:
  double sample_time;

  lineofsight _lineofsight;  // path following using LOS
};

#endif /* _PLANNER_H_ */