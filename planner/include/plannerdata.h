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
  Eigen::Vector3d setpoint;
};

#endif /*_PLANNERDATA_H_*/