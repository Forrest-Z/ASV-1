/*
*******************************************************************************
* thrustallocation.h:
* function for control allocation based on Quadratic programming, using
* Mosek solver API. Normally, thrust alloation is used in the fully-actuated
* control system
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _THRUSTALLOCATION_H_
#define _THRUSTALLOCATION_H_
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "controllerdata.h"
#include "mosek.h" /* Include the MOSEK definition file. */
// #include "realtimedata.h"
// #include "timecounter.hpp"

#define QP_THREADS_USED 1

// static void MSKAPI printstr(void *handle, const char str[]) {  //
// printf("%s", str);  // } /* printstr */

class thrustallocation {
 public:
  explicit thrustallocation(int _m, int _n)
      : m(_m), n(_n), Omega(m, m), Q_deltau(m, m) {}
  thrustallocation() = delete;
  ~thrustallocation() {}

 private:
  // constants of the first vessel
  int m;
  int n;
  int numvar;
  int num_constraints;  // the number of constraints

  std::vector<tunnelthrusterdata>
      v_tunnelthrusterdata;  // constant value of all tunnel thrusters
  std::vector<azimuththrusterdata>
      v_azimuththrusterdata;  // constant value of all tunnel thrusters

  // real time constraints of bow thruster
  int index_twice_bow;
  Eigen::Vector2d upper_delta_alpha_bow;
  Eigen::Vector2d lower_delta_alpha_bow;
  Eigen::Vector2d upper_delta_u_bow;
  Eigen::Vector2d lower_delta_u_bow;
  // real time constraints on azimuth thruster
  double upper_delta_alpha_left;
  double lower_delta_alpha_left;
  double upper_delta_u_left;
  double lower_delta_u_left;
  double upper_delta_alpha_right;
  double lower_delta_alpha_right;
  double upper_delta_u_right;
  double lower_delta_u_right;

  // quadratic objective
  Eigen::Matrix3d Q;
  Eigen::MatrixXd Omega;
  // real time Quadratic Objective in QP
  Eigen::MatrixXd Q_deltau;
  Eigen::VectorXd g_deltau;
  Eigen::Vector3d d_rho;  // derivative of rho term

  // real time constraint matrix in QP
  Eigen::Matrix3d B_alpha;
  Eigen::Matrix3d d_Balpha_u;  // Jocobian matrix of Balpha times u
  Eigen::Vector3d b;

  // real time physical variable in thruster allocation
  Eigen::Vector3d delta_alpha;  // rad
  Eigen::Vector3d delta_u;
  // linearized parameters
  double derivative_dx;  // step size of the derivative

  // // parameters for Mosek API
  // const MSKint32t aptrb[9], aptre[9], asub[21];
  // double aval[21];
  // /* Bounds on constraints. */
  // const MSKboundkeye bkc[3];
  // double blc[3];
  // double buc[3];
  // /* Bounds on variables. */
  // const MSKboundkeye bkx[9];
  // double blx[9];
  // double bux[9];

  // // objective g
  // double g[9];

  // // The lower triangular part of the quadratic objective Q matrix in the
  // // objective is specified.
  // const MSKint32t qsubi[9];
  // const MSKint32t qsubj[9];
  double qval[9];
  // array to store the optimization results
  Eigen::Matrix<double, 9, 2> results;

  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  MSKrescodee r;
};

#endif /* _THRUSTALLOCATION_H_*/