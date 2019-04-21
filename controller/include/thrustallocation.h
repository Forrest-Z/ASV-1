/*
*******************************************************************************
* thrustallocation.h:
* function for control allocation based on Quadratic programming, using
* Mosek solver API. Normally, thrust alloation is used in the fully-actuated
* control system.
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

template <class T>
void convertstdvector2mosekarray(const std::vector<T> &_v, T *_c) {
  // for (std::size_t i = 0; i < _v.size(); ++i)
  // *(_c + i) = _v[i];
  *(_c + 2) = 0;
}

// static void MSKAPI printstr(void *handle, const char str[]) {  //
// printf("%s", str);  // } /* printstr */

// m: # of all thrusters on the vessel
// n: # of dimension of control space
template <int m = 3, int n = 3>
class thrustallocation {
 public:
  explicit thrustallocation(
      const thrustallocationdata &_thrustallocationdata,
      const std::vector<tunnelthrusterdata> &_v_tunnelthrusterdata,
      const std::vector<azimuththrusterdata> &_v_azimuththrusterdata)
      : num_tunnel(_thrustallocationdata.num_tunnel),
        num_azimuth(_thrustallocationdata.num_azimuth),
        num_mainrudder(_thrustallocationdata.num_mainrudder),
        numvar(2 * m + n),
        num_constraints(n),
        index_thrusters(_thrustallocationdata.index_thrusters),
        v_tunnelthrusterdata(_v_tunnelthrusterdata),
        v_azimuththrusterdata(_v_azimuththrusterdata),
        lx(Eigen::Matrix<double, m, 1>::Zero()),
        ly(Eigen::Matrix<double, m, 1>::Zero()),
        upper_delta_alpha(Eigen::Matrix<double, m, 1>::Zero()),
        lower_delta_alpha(Eigen::Matrix<double, m, 1>::Zero()),
        upper_delta_u(Eigen::Matrix<double, m, 1>::Zero()),
        lower_delta_u(Eigen::Matrix<double, m, 1>::Zero()),
        Q(Eigen::Matrix<double, n, n>::Zero()),
        Omega(Eigen::Matrix<double, m, m>::Zero()),
        Q_deltau(Eigen::Matrix<double, m, m>::Zero()),
        g_deltau(Eigen::Matrix<double, m, 1>::Zero()),
        d_rho(Eigen::Matrix<double, m, 1>::Zero()),
        B_alpha(Eigen::Matrix<double, n, m>::Zero()),
        d_Balpha_u(Eigen::Matrix<double, n, m>::Zero()),
        b(Eigen::Matrix<double, n, 1>::Zero()),
        delta_alpha(Eigen::Matrix<double, m, 1>::Zero()),
        delta_u(Eigen::Matrix<double, m, 1>::Zero()),
        derivative_dx(1e-5) {
    initializethrusterallocation();
  }

  // explicit thrustallocation(
  //     const thrustallocationdata &_thrustallocationdata,
  //     const std::vector<tunnelthrusterdata> &_v_tunnelthrusterdata,
  //     const std::vector<azimuththrusterdata> &_v_azimuththrusterdata,
  //     const std::vector<ruddermaindata> &_v_ruddermaindata) {}
  thrustallocation() = delete;
  ~thrustallocation() {}

 private:
  const int num_tunnel;
  const int num_azimuth;
  const int num_mainrudder;
  const int numvar;
  const int num_constraints;

  // types of each thruster
  std::vector<int> index_thrusters;
  // constant value of all tunnel thrusters
  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  // constant value of all tunnel thrusters
  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  // constant value of all tunnel thrusters
  std::vector<ruddermaindata> v_ruddermaindata;

  // location of each thruster
  Eigen::Matrix<double, m, 1> lx;
  Eigen::Matrix<double, m, 1> ly;
  // real time constraints of each thruster
  Eigen::Matrix<double, m, 1> upper_delta_alpha;
  Eigen::Matrix<double, m, 1> lower_delta_alpha;
  Eigen::Matrix<double, m, 1> upper_delta_u;
  Eigen::Matrix<double, m, 1> lower_delta_u;
  // quadratic objective
  Eigen::Matrix<double, n, n> Q;
  Eigen::Matrix<double, m, m> Omega;
  Eigen::Matrix<double, m, m> Q_deltau;
  // linear objective
  Eigen::Matrix<double, m, 1> g_deltau;
  Eigen::Matrix<double, m, 1> d_rho;

  // real time constraint matrix in QP (equality constraint)
  Eigen::Matrix<double, n, m> B_alpha;
  Eigen::Matrix<double, n, m> d_Balpha_u;  // Jocobian matrix of Balpha times u
  Eigen::Matrix<double, n, 1> b;

  // real time physical variable in thruster allocation
  Eigen::Matrix<double, m, 1> delta_alpha;  // rad
  Eigen::Matrix<double, m, 1> delta_u;      // N
  // linearized parameters
  double derivative_dx;  // step size of the derivative

  // parameters for Mosek API
  MSKint32t aptrb[2 * m + n], aptre[2 * m + n], asub[6 * m + n];
  double aval[6 * m + n];
  /* Bounds on constraints. */
  MSKboundkeye bkc[n];
  double blc[n];
  double buc[n];
  /* Bounds on variables. */
  MSKboundkeye bkx[2 * m + n];
  double blx[2 * m + n];
  double bux[2 * m + n];

  // objective g
  double g[2 * m + n];

  // The lower triangular part of the quadratic objective Q matrix in the
  // objective is specified.
  MSKint32t qsubi[2 * m + n];
  MSKint32t qsubj[2 * m + n];
  double qval[2 * m + n];
  // array to store the optimization results
  Eigen::Matrix<double, 2 * m + n, 1> results;

  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  MSKrescodee r;

  void initializethrusterallocation() {
    for (int i = 0; i != num_tunnel; ++i) {
      lx(i) = v_tunnelthrusterdata[i].lx;
      ly(i) = v_tunnelthrusterdata[i].ly;
    }
    for (int i = 0; i != num_azimuth; ++i) {
      lx(num_tunnel + i) = v_azimuththrusterdata[i].lx;
      ly(num_tunnel + i) = v_azimuththrusterdata[i].ly;
    }
    Q.setZero();

    std::cout << m << std::endl;
    std::cout << n << std::endl;
    // Omega.setZero();
    // Q_deltau.setZero();
    // g_deltau.setZero();
    // d_rho.setZero();
    // B_alpha.setZero();
    // d_Balpha_u.setZero();
    // b.setZero();
    // delta_alpha.setZero();
    // delta_u.setZero();
    // results.setZero();
    // initializeQuadraticObjective();

    // calculaterotation(_realtimevessel);
    // // update BalphaU
    // _realtimevessel.BalphaU =
    //     calculateBalphau(_realtimevessel.alpha, _realtimevessel.u);
    // initializeMosekAPI(_vessel_second);
  }

  void initializeQuadraticObjective() {
    Q(0, 0) = 1000;
    Q(1, 1) = 1000;
    Q(2, 2) = 1000;
    Omega(0, 0) = 1;
    Omega(1, 1) = 20;
    Omega(2, 2) = 20;
    qval[3] = Omega(0, 0);
    qval[4] = Omega(1, 1);
    qval[5] = Omega(2, 2);
    qval[6] = Q(0, 0);
    qval[7] = Q(1, 1);
    qval[8] = Q(2, 2);
  }
};

#endif /* _THRUSTALLOCATION_H_*/