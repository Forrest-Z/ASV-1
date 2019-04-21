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
#include <vector>

const int num_thrusters = 3;     // m: # of all thrusters on the vessel
const int num_controlspace = 3;  // n: # of dimension of control space

// real-time data in the controller
struct controllerRTdata {};

struct thrustallocationdata {
  const int num_tunnel;      // # of tunnel thruster
  const int num_azimuth;     // # of azimuth thruster
  const int num_mainrudder;  // # of main thruster with rudder
  const std::vector<int> index_thrusters;
};

// constant data of tunnel thruster, index = 1
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

// constant data of azimuth thruster, index = 2
// Azimuth thruster can be used for fixed thruster, with a fixed alpha
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

// constant data of main propeller with rudder, index = 3
struct ruddermaindata {
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