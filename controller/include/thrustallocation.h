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
  explicit thrustallocation(
      const thrustallocationdata &_thrustallocationdata,
      const std::vector<tunnelthrusterdata> &_v_tunnelthrusterdata,
      const std::vector<azimuththrusterdata> &_v_azimuththrusterdata)
      : m(_thrustallocationdata.m),
        n(_thrustallocationdata.n),
        numvar(2 * _thrustallocationdata.m + _thrustallocationdata.n),
        num_constraints(_thrustallocationdata.n),
        index_thrusters(_thrustallocationdata.index_thrusters),
        v_tunnelthrusterdata(_v_tunnelthrusterdata),
        v_azimuththrusterdata(_v_azimuththrusterdata),
        lx(Eigen::VectorXd::Zero(m)),
        ly(Eigen::VectorXd::Zero(m)),
        upper_delta_alpha(Eigen::VectorXd::Zero(m)),
        lower_delta_alpha(Eigen::VectorXd::Zero(m)),
        upper_delta_u(Eigen::VectorXd::Zero(m)),
        lower_delta_u(Eigen::VectorXd::Zero(m)),
        Q(Eigen::Matrix3d::Zero()),
        Omega(Eigen::MatrixXd::Zero(m, m)),
        Q_deltau(Eigen::MatrixXd::Zero(m, m)) {}
  thrustallocation() = delete;
  ~thrustallocation() {}

 private:
  int m;
  int n;
  int numvar;
  int num_constraints;

  // types of each thruster
  Eigen::VectorXd index_thrusters;
  // constant value of all tunnel thrusters
  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  // constant value of all tunnel thrusters
  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  // location of each thruster
  Eigen::VectorXd lx;
  Eigen::VectorXd ly;
  // real time constraints of each thruster
  Eigen::VectorXd upper_delta_alpha;
  Eigen::VectorXd lower_delta_alpha;
  Eigen::VectorXd upper_delta_u;
  Eigen::VectorXd lower_delta_u;
  // quadratic objective
  Eigen::Matrix3d Q;
  Eigen::MatrixXd Omega;
  Eigen::MatrixXd Q_deltau;
  // linear objective
  Eigen::VectorXd g_deltau;
  Eigen::VectorXd d_rho;

  // real time constraint matrix in QP (equality constraint)
  Eigen::MatrixXd B_alpha;
  Eigen::Matrix3d d_Balpha_u;  // Jocobian matrix of Balpha times u
  Eigen::Vector3d b;

  // real time physical variable in thruster allocation
  Eigen::Vector3d delta_alpha;  // rad
  Eigen::Vector3d delta_u;
  // linearized parameters
  double derivative_dx;  // step size of the derivative

  // parameters for Mosek API
  Eigen::ArrayXi _aptrb, _aptre, _asub;
  Eigen::ArrayXd _aval;
  MSKint32t *aptrb, *aptre, *asub;
  double *aval;
  /* Bounds on constraints. */
  MSKboundkeye bkc[3];
  double blc[3];
  double buc[3];
  /* Bounds on variables. */
  MSKboundkeye bkx[9];
  double blx[9];
  double bux[9];

  // objective g
  double g[9];

  // The lower triangular part of the quadratic objective Q matrix in the
  // objective is specified.
  MSKint32t qsubi[9];
  MSKint32t qsubj[9];
  double qval[9];
  // array to store the optimization results
  Eigen::Matrix<double, 9, 2> results;

  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  MSKrescodee r;

  // void initializethrusterallocation(const vessel_second &_vessel_second,
  //                                   realtimevessel_second &_realtimevessel) {
  //   lx << _vessel_second.bow_x, _vessel_second.left_x,
  //   _vessel_second.right_x; ly << _vessel_second.bow_y,
  //   _vessel_second.left_y, _vessel_second.right_y; Q.setZero();
  //   Omega.setZero();
  //   Q_deltau.setZero();
  //   g_deltau.setZero();
  //   d_rho.setZero();
  //   B_alpha.setZero();
  //   d_Balpha_u.setZero();
  //   b.setZero();
  //   delta_alpha.setZero();
  //   delta_u.setZero();
  //   results.setZero();
  //   initializeQuadraticObjective();

  //   calculaterotation(_realtimevessel);
  //   // update BalphaU
  //   _realtimevessel.BalphaU =
  //       calculateBalphau(_realtimevessel.alpha, _realtimevessel.u);
  //   initializeMosekAPI(_vessel_second);
  // }

  // void initializeQuadraticObjective() {
  //   Q(0, 0) = 1000;
  //   Q(1, 1) = 1000;
  //   Q(2, 2) = 1000;
  //   Omega(0, 0) = 1;
  //   Omega(1, 1) = 20;
  //   Omega(2, 2) = 20;
  //   qval[3] = Omega(0, 0);
  //   qval[4] = Omega(1, 1);
  //   qval[5] = Omega(2, 2);
  //   qval[6] = Q(0, 0);
  //   qval[7] = Q(1, 1);
  //   qval[8] = Q(2, 2);
  // }
};

#endif /* _THRUSTALLOCATION_H_*/