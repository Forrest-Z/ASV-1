/*
***********************************************************************
* planner.h:
* function for motion planning of USV
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

  planner &followcircle(plannerRTdata &_plannerRTdata,
                        const Eigen::Vector3d &_vesselstate,
                        const Eigen::Vector2d &_startposition,
                        const Eigen::Vector2d &_endposition, double _radius) {
    double steplength = _plannerRTdata.v_setpoint(0) * sample_time;

    Eigen::Vector2d delta_pos = _endposition - _startposition;
    double length =
        std::sqrt(std::pow(delta_pos(0), 2) + std::pow(delta_pos(1), 2));

    // orientation of vector points from starting to the ending
    double thetaK = std::atan(delta_pos(1) / delta_pos(0));
    if (delta_pos(0) < 0) thetaK += M_PI;

    if (length > 2 * _radius) {
      _plannerRTdata.waypoint0 = _startposition;
      _plannerRTdata.waypoint1 = _endposition;
    } else {
      double gamma = std::acos(length / (2 * _radius));

      _startheading(0) = restrictheadingangle(thetaK + gamma + 0.5 * M_PI);
      _startheading(1) = restrictheadingangle(thetaK + gamma - 0.5 * M_PI);
      _startheading(2) = restrictheadingangle(thetaK - gamma - 0.5 * M_PI);
      _startheading(3) = restrictheadingangle(thetaK - gamma + 0.5 * M_PI);
    }
    return *this;
  }

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
  Eigen::Matrix<double, Eigen::Dynamic, 2> waypoints;
  lineofsight _lineofsight;  // path following using LOS

  // restrict heading angle (0-2PI) to (-PI ~ PI)
  double restrictheadingangle(double _heading) noexcept {
    double heading = 0.0;
    if (_heading > M_PI)
      heading = _heading - 2 * M_PI;
    else if (_heading < -M_PI)
      heading = _heading + 2 * M_PI;
    else
      heading = _heading;
    return heading;
  }
};

#endif /* _PLANNER_H_ */