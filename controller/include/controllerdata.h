/*
*******************************************************************************
* controllerdata.h:
* function for control allocation based on Quadratic programming, using
* Mosek solver API. Normally, thrust alloation is used in the fully-actuated
* control system
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _CONTROLLERDATA_H_
#define _CONTROLLERDATA_H_

#include <Eigen/Core>
#include <Eigen/Dense>

struct thrustallocationdata {
  int m;
  int n;
  Eigen::VectorXd index_thrusters;  // types of each thruster
};

struct tunnelthrusterdata {
  double lx;  // m
  double ly;  // m
  double K_positive;
  double K_negative;
  double max_delta_rotation;
  double max_rotation;
  double max_thrust_positive;
  double max_thrust_negative;
};

struct azimuththrusterdata {
  double lx;                  // m
  double ly;                  // m
  double K;                   //
  double max_delta_rotation;  // rpm
  double max_rotation;        // rpm
  double min_rotation;        // rpm
  double max_delta_alpha;     // rad
  double max_alpha;           // rad
  double min_alpha;           // rad
  double max_thrust;          // N
  double min_thrust;          // N
};

#endif /* _CONTROLLERDATA_H_ */