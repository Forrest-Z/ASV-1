/*
*******************************************************************************
* plannerdata.h:
* define the data struct used in the controller
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _PLANNERDATA_H_
#define _PLANNERDATA_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

// real time data in planner
struct plannerRTdata {
  Eigen::Vector3d setpoint;    // x, y, theta in the global coordinate
  Eigen::Vector3d v_setpoint;  // u, v, r in the body coordinate
  Eigen::Vector2d waypoint0;   // x, y in the global coordinate
  Eigen::Vector2d waypoint1;   // x, y in the global coordinate
  Eigen::Vector3d command;     // command from joystick (human)
};

struct plannerdata {
  double sample_time;
  double los_radius;
  double los_capture_radius;
};

#endif /*_PLANNERDATA_H_*/