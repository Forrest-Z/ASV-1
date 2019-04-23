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
#include <cmath>
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
      controllerRTdata &_RTdata,
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
        lx(vectormd::Zero()),
        ly(vectormd::Zero()),
        upper_delta_alpha(vectormd::Zero()),
        lower_delta_alpha(vectormd::Zero()),
        upper_delta_u(vectormd::Zero()),
        lower_delta_u(vectormd::Zero()),
        Q(matrixnnd::Zero()),
        Omega(matrixmmd::Zero()),
        Q_deltau(matrixmmd::Zero()),
        g_deltau(vectormd::Zero()),
        d_rho(vectormd::Zero()),
        B_alpha(matrixnmd::Zero()),
        d_Balpha_u(matrixnmd::Zero()),
        b(vectornd::Zero()),
        delta_alpha(vectormd::Zero()),
        delta_u(vectormd::Zero()),
        derivative_dx(1e-5),
        results(Eigen::Matrix<double, 2 * m + n, 1>::Zero()) {
    initializethrusterallocation(_RTdata);
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
  matrixnnd Q;
  matrixmmd Omega;
  matrixmmd Q_deltau;
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

  void initializethrusterallocation(controllerRTdata &_RTdata) {
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

    calculaterotation(_RTdata);
    // update BalphaU
    _RTdata.BalphaU = calculateBalphau(_RTdata.alpha, _RTdata.u);
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

  // calculate the contraints of tunnel thruster
  // depend on the desired force in the Y direction or Mz direction
  void calculateconstrains_tunnelthruster(const controllerRTdata &_RTdata,
                                          double _desired_Mz) {
    for (int i = 0; i != num_tunnel; ++i) {
      int _maxdeltar = v_tunnelthrusterdata[i].max_delta_rotation;
      double _Kp = v_tunnelthrusterdata[i].K_positive;
      double _Kn = v_tunnelthrusterdata[i].K_negative;
      if (0 < _RTdata.rotation(i) && _RTdata.rotation(i) <= _maxdeltar) {
        // specify the first case
        if (_desired_Mz > 0) {
          upper_delta_alpha(i) = 0;
          lower_delta_alpha(i) = 0;
          lower_delta_u(i) = -_RTdata.u(i);
          upper_delta_u(i) =
              _Kp * std::pow(_RTdata.rotation(i) + _maxdeltar, 2) -
              _RTdata.u(i);
        } else {
          upper_delta_alpha(i) = -M_PI;
          lower_delta_alpha(i) = -M_PI;
          lower_delta_u(i) = -_RTdata.u(i);
          upper_delta_u(i) = _Kn *
                                 std::pow(_RTdata.rotation(i) - _maxdeltar,
                                          2)(_RTdata.rotation(i) - _maxdeltar) *
                                 (_RTdata.rotation(i) - _maxdeltar) -
                             _RTdata.u(i);
        }

      } else if (-_maxdeltar <= _RTdata.rotation(i) &&
                 _RTdata.rotation(i) < 0) {
        if (_desired_Mz > 0) {
          // specify the second case
          upper_delta_alpha(i) = M_PI;
          lower_delta_alpha(i) = M_PI;
          lower_delta_u(i) = -_RTdata.u(i);
          upper_delta_u(i) = _Kp * (_RTdata.rotation(i) + _maxdeltar) *
                                 (_RTdata.rotation(i) + _maxdeltar) -
                             _RTdata.u(i);
        } else {
          // specify the first case
          upper_delta_alpha(i) = 0;
          lower_delta_alpha(i) = 0;
          lower_delta_u(i) = -_RTdata.u(i);
          upper_delta_u(i) = _Kn * (_RTdata.rotation(i) - _maxdeltar) *
                                 (_RTdata.rotation(i) - _maxdeltar) -
                             _RTdata.u(i);
        }

      } else if (_RTdata.rotation(i) > _maxdeltar) {
        lower_delta_alpha(i) = 0;
        upper_delta_alpha(i) = 0;
        upper_delta_u(i) = std::min(
            v_tunnelthrusterdata[i].max_thrust_positive - _RTdata.u(i),
            _Kp * std::pow(_RTdata.rotation(i) + _maxdeltar, 2) - _RTdata.u(i));
        lower_delta_u(i) =
            _Kp * std::pow(_RTdata.rotation(i) - _maxdeltar, 2) - _RTdata.u(i);

      } else {
        lower_delta_alpha(i) = 0;
        upper_delta_alpha(i) = 0;
        upper_delta_u(i) = std::min(
            _Kn * std::pow(_RTdata.rotation(i) - _maxdeltar, 2) - _RTdata.u(i),
            v_tunnelthrusterdata[i].max_thrust_negative - _RTdata.u(i));
        lower_delta_u(i) =
            _Kn * std::pow(_RTdata.rotation(i) + _maxdeltar, 2) - _RTdata.u(i);
      }
    }
  }

  // calculate the consraints of azimuth thruster
  void calculateconstrains_azimuth(
      const realtimevessel_first &_realtimevessel) {
    // specify constriants on left azimuth
    /* contraints on the increment of angle */
    upper_delta_alpha_left =
        std::min(max_delta_alpha_azimuth,
                 max_alpha_azimuth_left - _realtimevessel.alpha(1));
    lower_delta_alpha_left =
        std::max(-max_delta_alpha_azimuth,
                 min_alpha_azimuth_left - _realtimevessel.alpha(1));
    /* contraints on the increment of thrust */
    double thrust_azimuth_left =
        K_left * _realtimevessel.rotation(1) * _realtimevessel.rotation(1);
    upper_delta_u_left =
        std::min(
            K_left *
                (_realtimevessel.rotation(1) + max_delta_rotation_azimuth) *
                (_realtimevessel.rotation(1) + max_delta_rotation_azimuth),
            max_thrust_azimuth_left) -
        thrust_azimuth_left;

    lower_delta_u_left =
        std::max(
            K_left *
                (_realtimevessel.rotation(1) - max_delta_rotation_azimuth) *
                (_realtimevessel.rotation(1) - max_delta_rotation_azimuth),
            min_thrust_azimuth_left) -
        thrust_azimuth_left;

    // specify constraints on right azimuth
    /* contraints on the increment of angle */
    upper_delta_alpha_right =
        std::min(max_delta_alpha_azimuth,
                 max_alpha_azimuth_right - _realtimevessel.alpha(2));
    lower_delta_alpha_right =
        std::max(-max_delta_alpha_azimuth,
                 min_alpha_azimuth_right - _realtimevessel.alpha(2));
    /* contraints on the increment of thrust */
    double thrust_azimuth_right =
        K_right * _realtimevessel.rotation(2) * _realtimevessel.rotation(2);
    upper_delta_u_right =
        std::min(
            K_right *
                (_realtimevessel.rotation(2) + max_delta_rotation_azimuth) *
                (_realtimevessel.rotation(2) + max_delta_rotation_azimuth),
            max_thrust_azimuth_right) -
        thrust_azimuth_right;

    lower_delta_u_right =
        std::max(
            K_right *
                (_realtimevessel.rotation(2) - max_delta_rotation_azimuth) *
                (_realtimevessel.rotation(2) - max_delta_rotation_azimuth),
            min_thrust_azimuth_right) -
        thrust_azimuth_right;
  }

  // calcuate rotation speed of each thruster based on thrust
  void calculaterotation(controllerRTdata &_RTdata) {
    // bow thruster
    for (int i = 0; i != num_tunnel; ++i) {
      int t_rotation = 0;
      if (_RTdata.alpha(i) < 0) {
        t_rotation = static_cast<int>(
            sqrt(abs(_RTdata.u(i)) / v_tunnelthrusterdata[i].K_negative));
        if (t_rotation == 0) {
          _RTdata.rotation(i) = -1;  // prevent zero
          _RTdata.u(i) = v_tunnelthrusterdata[i].K_negative;
        } else
          _RTdata.rotation(i) = -t_rotation;

      } else {
        t_rotation = static_cast<int>(
            sqrt(abs(_RTdata.u(i)) / v_tunnelthrusterdata[i].K_positive));

        if (t_rotation == 0) {
          _RTdata.rotation(i) = 1;  // prevent zero
          _RTdata.u(i) = v_tunnelthrusterdata[i].K_positive;
        } else
          _RTdata.rotation(i) = t_rotation;
      }
    }

    // azimuth thruster
    for (int j = 0; j != num_azimuth; ++j) {
      int t_rotation = 0;
      int index_azimuth = j + num_tunnel;

      t_rotation = static_cast<int>(
          sqrt(abs(_RTdata.u(index_azimuth)) / v_azimuththrusterdata[j].K));
      if (t_rotation == 0) {
        _RTdata.rotation(index_azimuth) = 1;
        _RTdata.u(index_azimuth) = v_azimuththrusterdata[j].K;
      } else
        _RTdata.rotation(index_azimuth) = t_rotation;
    }
  }

  // calculate Balpha as function of alpha
  matrixnmd calculateBalpha(const vectormd &t_alpha) {
    matrixnmd _B_alpha = matrixnmd::Zero();
    double angle = 0;
    double t_cos = 0;
    double t_sin = 0;
    for (int i = 0; i != m; ++i) {
      angle = t_alpha(i);
      t_cos = cos(angle);
      t_sin = sin(angle);
      _B_alpha(0, i) = t_cos;
      _B_alpha(1, i) = t_sin;
      _B_alpha(2, i) = -ly(i) * t_cos + lx(i) * t_sin;
    }
    return _B_alpha;
  }
  // calculate the rho term in thruster allocation
  double calculateRhoTerm(const vectormd &t_alpha, double epsilon = 0.01,
                          double rho = 10) {
    auto _B_alpha = calculateBalpha(t_alpha);
    matrixnnd BBT = _B_alpha * _B_alpha.transpose();
    return rho / (epsilon + BBT.determinant());
  }
  // calculate Jacobian using central difference
  void calculateJocobianRhoTerm(const vectormd &t_alpha) {
    for (int i = 0; i != m; ++i) {
      auto alpha_plus = t_alpha;
      auto alpha_minus = t_alpha;
      alpha_plus(i) += derivative_dx;
      alpha_minus(i) -= derivative_dx;
      d_rho(i) =
          (calculateRhoTerm(alpha_plus) - calculateRhoTerm(alpha_minus)) /
          (2 * derivative_dx);
    }
  }
  // calculate the Balpha u term
  vectornd calculateBalphau(const vectormd &t_alpha, const vectormd &t_u) {
    return calculateBalpha(t_alpha) * t_u;
  }
  // calculate derivative of Balpha times u
  void calculateJocobianBalphaU(const vectormd &t_alpha, const vectormd &t_u) {
    for (int i = 0; i != m; ++i) {
      auto alpha_plus = t_alpha;
      auto alpha_minus = t_alpha;
      alpha_plus(i) += derivative_dx;
      alpha_minus(i) -= derivative_dx;
      d_Balpha_u.col(i) = (calculateBalphau(alpha_plus, t_u) -
                           calculateBalphau(alpha_minus, t_u)) /
                          (2 * derivative_dx);
    }
  }
};

#endif /* _THRUSTALLOCATION_H_*/