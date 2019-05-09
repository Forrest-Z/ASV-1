/*
*******************************************************************************
* controllerdata.h:
* define the data struct used in the controller
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

enum CONTROLMODE {
  MANUAL = 0,   // manual controller
  HEADINGONLY,  // heading only controller
  AUTOMATIC     // automatic control
};

enum WINDCOMPENSATION {
  WINDOFF = 0,  // turn off the wind compenstation
  WINDON        // turn on the wind compenstation
};

// indicator in the controller
struct controllerdata {
  double sample_time;  // sample time of controller((unit: second)),
  CONTROLMODE controlmode;
  WINDCOMPENSATION windcompensation;
};

// real-time data in the controller
template <int m, int n = 3>
struct controllerRTdata {
  // Fx, Fy, Mz (desired force) in the body coordinate
  Eigen::Matrix<double, n, 1> tau;
  // Fx, Fy, Mz (estimated generalized thrust) in the body-fixed coordinates
  Eigen::Matrix<double, n, 1> BalphaU;
  // N, estimated thrust of all propellers
  Eigen::Matrix<double, m, 1> u;
  // rpm, rotation of all propellers
  Eigen::Matrix<int, m, 1> rotation;
  // rad, angle of all propellers
  Eigen::Matrix<double, m, 1> alpha;
  // deg, angle of all propellers
  Eigen::Matrix<int, m, 1> alpha_deg;
};

// real time wind compensation
struct windestimation {
  // Fx, Fy, Mz (wind force) in the body coordinate
  Eigen::Matrix<double, 3, 1> load;
  // wind direction and speed in body
  Eigen::Matrix<double, 2, 1> wind_body;  // direction, speed
  // wind direction and speed in global
  Eigen::Matrix<double, 2, 1> wind_global;  // direction, speed
};

struct thrustallocationdata {
  const int num_tunnel;      // # of tunnel thruster
  const int num_azimuth;     // # of azimuth thruster
  const int num_mainrudder;  // # of main thruster with rudder
  const std::vector<int> index_thrusters;
};

// constant data of tunnel thruster, index = 1
struct tunnelthrusterdata {
  const double lx;  // m
  const double ly;  // m
  const double K_positive;
  const double K_negative;
  const int max_delta_rotation;
  const int max_rotation;
  const double max_thrust_positive;
  const double max_thrust_negative;
};

// constant data of azimuth thruster, index = 2
// Azimuth thruster can be used for fixed thruster, with a fixed alpha
struct azimuththrusterdata {
  const double lx;               // m
  const double ly;               // m
  const double K;                //
  const int max_delta_rotation;  // rpm
  const int max_rotation;        // rpm
  const int min_rotation;        // rpm
  const double max_delta_alpha;  // rad
  const double max_alpha;        // rad
  const double min_alpha;        // rad
  const double max_thrust;       // N
  const double min_thrust;       // N
};

// constant data of main propeller with rudder, index = 3
struct ruddermaindata {
  const double lx;                  // m
  const double ly;                  // m
  const double K;                   //
  const double max_delta_rotation;  // rpm
  const double max_rotation;        // rpm
  const double min_rotation;        // rpm
  const double max_delta_alpha;     // rad
  const double max_alpha;           // rad
  const double min_alpha;           // rad
  const double max_thrust;          // N
  const double min_thrust;          // N
};

// quasi-static data of pid controller
struct pidcontrollerdata {
  double P;
  double I;
  double D;
  const double allowed_error;
  const double min_output;
  const double max_output;
};

#endif /* _CONTROLLERDATA_H_ */