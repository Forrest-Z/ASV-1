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

#include <cmath>
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

  Eigen::MatrixXd followcircle(const Eigen::Vector2d &_startposition,
                               const Eigen::Vector2d &_endposition,
                               double _radius, double _vesselheading,
                               double _desiredspeed) {
    Eigen::Vector2d delta_pos = _endposition - _startposition;
    double length = computevectorlength(delta_pos(1), delta_pos(0));
    double thetaK = computevectororientation(delta_pos(1), delta_pos(0));
    Eigen::MatrixXd waypoints(2, 2);
    if (length > 2 * _radius) {
      waypoints.col(0) = _startposition;
      waypoints.col(1) = _endposition;
    } else {
      double gamma = std::acos(length / (2 * _radius));
      int n = static_cast<int>(std::floor((M_PI - 2 * gamma) * 10));
      waypoints.resize(Eigen::NoChange, n);

      double _clockwise_angle = thetaK + gamma;
      double _anticlockwise_angle = thetaK - gamma;

      double delta_clockwise_angle = std::abs(
          restrictheadingangle(_vesselheading - _clockwise_angle + 0.5 * M_PI));
      double delta_anticlockwise_angle = std::abs(restrictheadingangle(
          _vesselheading - _anticlockwise_angle - 0.5 * M_PI));
      if (delta_clockwise_angle < delta_anticlockwise_angle) {
        // follow the clockwise circle
        waypoints = generatecirclepoints(
            _startposition + _radius * computevectorlocation(_clockwise_angle),
            _radius, n, _clockwise_angle - M_PI, _anticlockwise_angle);
      } else {
        // follow the anti-clockwise circle
        waypoints = generatecirclepoints(
            _startposition +
                _radius * computevectorlocation(_anticlockwise_angle),
            _radius, n, _clockwise_angle, _anticlockwise_angle + M_PI);
      }
    }
    return waypoints;
  }

  planner &initializewaypoint(plannerRTdata &_plannerRTdata,
                              const Eigen::MatrixXd &_wpset) {
    _plannerRTdata.waypoint0 = _wpset.col(0);
    _plannerRTdata.waypoint1 = _wpset.col(1);
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
  planner &setconstantspeed(plannerRTdata &_plannerRTdata, double _forwardspeed,
                            double _headingrate, double _swayspeed = 0.0) {
    _plannerRTdata.v_setpoint(0) = _forwardspeed;
    _plannerRTdata.v_setpoint(1) = _swayspeed;
    _plannerRTdata.v_setpoint(2) = _headingrate;
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

  //
  Eigen::MatrixXd generatecirclepoints(double _radius, double _center_x,
                                       double _center_y, double _start_angle,
                                       double _end_angle, int n) {
    Eigen::VectorXd angles =
        Eigen::VectorXd::LinSpaced(n, _start_angle, _end_angle);
    Eigen::MatrixXd waypoints_set(2, n);
    for (int i = 0; i != n; ++i) {
      waypoints_set(0, i) = _center_x + _radius * std::cos(angles(i));
      waypoints_set(1, i) = _center_y + _radius * std::sin(angles(i));
    }
    return waypoints_set;
  }
  //
  Eigen::MatrixXd generatecirclepoints(const Eigen::Vector2d &_center,
                                       double _radius, int n,
                                       double _start_angle, double _end_angle) {
    Eigen::VectorXd angles =
        Eigen::VectorXd::LinSpaced(n, _start_angle, _end_angle);
    Eigen::MatrixXd waypoints_set(2, n);
    for (int i = 0; i != n; ++i) {
      waypoints_set.col(i) =
          _center + _radius * computevectorlocation(angles(i));
    }
    return waypoints_set;
  }
  // orientation of vector points from starting to the ending
  double computevectororientation(double _vy, double _vx) {
    double thetaK = std::atan(_vy / _vx);
    if (_vx < 0) thetaK += M_PI;
    return thetaK;
  }
  // compute the vector length
  double computevectorlength(double _vy, double _vx) noexcept {
    return std::sqrt(std::pow(_vy, 2) + std::pow(_vx, 2));
  }
  // compute the
  Eigen::Vector2d computevectorlocation(double _angle) {
    return (Eigen::Vector2d() << std::cos(_angle), std::sin(_angle)).finished();
  }
};

#endif /* _PLANNER_H_ */