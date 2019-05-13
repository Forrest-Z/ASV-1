/*
*******************************************************************************
* controllerdata.h:
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
  Eigen::Vector3d setpoint;       // x, y, theta in the global coordinate
  Eigen::Vector3d setpoint_body;  // x, y, theta in the body-fixed coordinate
  Eigen::Vector3d command;        // command from joystick (human)
};

#endif /*_PLANNERDATA_H_*/