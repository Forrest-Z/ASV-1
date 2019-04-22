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

// static void MSKAPI printstr(void *handle, const char str[]) {  //
// printf("%s", str);  // } /* printstr */

// m: # of all thrusters on the vessel
// n: # of dimension of control space
template <int m, int n = 3>
class thrustallocation {
 public:
  explicit thrustallocation(
      controllerRTdata &_controllerRTdata,
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
        derivative_dx(1e-5),
        results(Eigen::Matrix<double, 2 * m + n, 1>::Zero()) {
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

      Omega(i, i) = 1;
    }
    for (int i = 0; i != num_azimuth; ++i) {
      int index_azimuth = num_tunnel + i;
      lx(index_azimuth) = v_azimuththrusterdata[i].lx;
      ly(index_azimuth) = v_azimuththrusterdata[i].ly;

      Omega(index_azimuth, index_azimuth) = 20;
    }

    // quadratic penality matrix for error
    Q(0, 0) = 1000;
    Q(1, 1) = 1000;
    Q(2, 2) = 1000;

    initializemosekvariables();

    calculaterotation(_realtimevessel);
    // update BalphaU
    _realtimevessel.BalphaU =
        calculateBalphau(_realtimevessel.alpha, _realtimevessel.u);
    initializeMosekAPI();
  }

  void initializemosekvariables() {
    int _mdouble = 2 * m;
    int _mquintuple = 6 * m;
    // assign value to the bounds on variables
    for (int i = 0; i != _mdouble; ++i) {
      bkx[i] = MSK_BK_RA;
      blx[i] = 0.0;
      bux[i] = 0.0;
    }
    for (int j = 0; j != n; ++j) {
      int index_n = j + _mdouble;
      bkx[index_n] = MSK_BK_FR;
      blx[index_n] = -MSK_INFINITY;
      bux[index_n] = +MSK_INFINITY;
    }

    // assign value to the linear constraints
    for (int i = 0; i != _mdouble; ++i) {
      int _itriple = 3 * i;
      aptrb[i] = _itriple;
      aptre[i] = _itriple + 3;

      for (int j = 0; j != 3; ++j) {
        asub[_itriple + j] = j;
        aval[_itriple + j] = 0.0;
      }
    }
    for (int j = 0; j != n; ++j) {
      aptrb[j + _mdouble] = _mquintuple + j;
      aptre[j + _mdouble] = _mquintuple + j + 1;
      asub[j + _mquintuple] = j;
      aval[j + _mquintuple] = 1.0;
      bkc[j] = MSK_BK_FX;
      blc[j] = 0;
      buc[j] = 0;
    }

    // assign value to the objective
    for (int i = 0; i != numvar; ++i) {
      qsubi[i] = i;
      qsubj[i] = i;
      g[i] = 0;
    }
    for (int i = 0; i != m; ++i) {
      qval[i] = 0;
    }
    for (int i = 0; i != m; ++i) {
      qval[i + m] = Omega(i, i);
    }
    for (int j = 0; j != n; ++j) {
      qval[j + 2 * m] = Q(j, j);
    }
  }

  void initializeMosekAPI() {
    /* Create the mosek environment. */
    r = MSK_makeenv(&env, NULL);
    /* Create the optimization task. */
    r = MSK_maketask(env, num_constraints, numvar, &task);
    // set up the threads used by mosek
    r = MSK_putintparam(task, MSK_IPAR_NUM_THREADS, QP_THREADS_USED);
    // r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
    r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, NULL);
    // append num_constrainsts empty contraints
    r = MSK_appendcons(task, num_constraints);
    // append numvar emtpy variables
    r = MSK_appendvars(task, numvar);
  }

  // calcuate rotation speed of each thruster based on thrust
  void calculaterotation(realtimevessel_first &_realtimevessel) {
    int t_rotation = 0;
    // bow thruster
    for (int i = 0; i != num_tunnel; ++i) {
    }

    if (_realtimevessel.alpha(0) < 0) {
      t_rotation = (int)(sqrt(abs(_realtimevessel.u(0)) / Kbar_negative));
      if (t_rotation == 0) {
        _realtimevessel.rotation(0) = -1;  // prevent zero
        _realtimevessel.u(0) = Kbar_negative;
      } else
        _realtimevessel.rotation(0) = -t_rotation;

    } else {
      t_rotation = (int)(sqrt(abs(_realtimevessel.u(0)) / Kbar_positive));
      if (t_rotation == 0) {
        _realtimevessel.rotation(0) = 1;  // prevent zero
        _realtimevessel.u(0) = Kbar_positive;
      } else
        _realtimevessel.rotation(0) = t_rotation;
    }

    // azimuth thruster
    for (int j = 0; j != num_azimuth; ++j) {
    }

    t_rotation = (int)(sqrt(abs(_realtimevessel.u(1)) / K_left));
    if (t_rotation == 0) {
      _realtimevessel.rotation(1) = 1;
      _realtimevessel.u(1) = K_left;
    } else
      _realtimevessel.rotation(1) = t_rotation;
    // azimuth thruster Right
    t_rotation = (int)(sqrt(abs(_realtimevessel.u(2)) / K_right));
    if (t_rotation == 0) {
      _realtimevessel.rotation(2) = 1;
      _realtimevessel.u(2) = K_right;
    } else
      _realtimevessel.rotation(2) = t_rotation;
  }
};

#endif /* _THRUSTALLOCATION_H_*/