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

enum ACTUATION {
  UNDERACTUATED = 0,  // underactuated usv
  FULLYACTUATED       // fully actuated usv
};

// indicator in the controller
struct controllerdata {
  double sample_time;  // sample time of controller((unit: second)),
  CONTROLMODE controlmode;
  WINDCOMPENSATION windstatus;
  ACTUATION index_actuation;
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
template <int n = 3>
struct windestimation {
  // Fx, Fy, Mz (wind force) in the body coordinate
  Eigen::Matrix<double, n, 1> load;
  // wind direction and speed in body
  Eigen::Matrix<double, 2, 1> wind_body;  // direction, speed
  // wind direction and speed in global
  Eigen::Matrix<double, 2, 1> wind_global;  // direction, speed
};

struct thrustallocationdata {
  int num_tunnel;      // # of tunnel thruster
  int num_azimuth;     // # of azimuth thruster
  int num_mainrudder;  // # of main thruster with rudder
  std::vector<int> index_thrusters;
};

// constant data of tunnel thruster, index = 1
struct tunnelthrusterdata {
  double lx;  // m
  double ly;  // m
  double K_positive;
  double K_negative;
  int max_delta_rotation;
  int max_rotation;
  double max_thrust_positive;  // positive value
  double max_thrust_negative;  // positive value
};

// constant data of azimuth thruster, index = 2
// Azimuth thruster can be used for fixed thruster, with a fixed alpha
struct azimuththrusterdata {
  double lx;               // m
  double ly;               // m
  double K;                //
  int max_delta_rotation;  // rpm
  int max_rotation;        // rpm
  int min_rotation;        // rpm
  double max_delta_alpha;  // rad
  double max_alpha;        // rad
  double min_alpha;        // rad
  double max_thrust;       // N, positive value
  double min_thrust;       // N, positive value
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

// quasi-static data of pid controller
struct pidcontrollerdata {
  double P;
  double I;
  double D;
  double allowed_error;
  double min_output;
  double max_output;
};

#endif /* _CONTROLLERDATA_H_ */