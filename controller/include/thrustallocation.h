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
#include "easylogging++.h"
#include "mosek.h" /* Include the MOSEK definition file. */

// # of thread used by mosek
#define QP_THREADS_USED 1

// m: # of all thrusters on the vessel
// n: # of dimension of control space
template <int m, int n = 3>
class thrustallocation {
  using vectormd = Eigen::Matrix<double, m, 1>;
  using vectormi = Eigen::Matrix<int, m, 1>;
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixnmd = Eigen::Matrix<double, n, m>;
  using matrixmmd = Eigen::Matrix<double, m, m>;
  using matrixnnd = Eigen::Matrix<double, n, n>;

 public:
  explicit thrustallocation(
      controllerRTdata<m, n> &_RTdata,
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
        derivative_dx(1e-6),
        results(Eigen::Matrix<double, 2 * m + n, 1>::Zero()) {
    initializethrusterallocation(_RTdata);
  }

  // explicit thrustallocation(
  //     const thrustallocationdata &_thrustallocationdata,
  //     const std::vector<tunnelthrusterdata> &_v_tunnelthrusterdata,
  //     const std::vector<azimuththrusterdata> &_v_azimuththrusterdata,
  //     const std::vector<ruddermaindata> &_v_ruddermaindata) {}
  thrustallocation() = delete;
  ~thrustallocation() {
    MSK_deletetask(&task);
    MSK_deleteenv(&env);
  }

  // perform the thrust allocation using QP solver (one step)
  void onestepthrustallocation(controllerRTdata<m, n> &_RTdata) {
    updateTAparameters(_RTdata);
    updateMosekparameters();
    onestepmosek();
    updateNextstep(_RTdata);
  }

  // modify penality for each error (heading-only controller)
  void setQ(controlmode _cm) {
    switch (_cm) {
      case MANUAL:
        for (int i = 0; i != n; ++i) Q(i, i) = 1000;
        break;
      case HEADINGONLY:
        Q(0, 0) = 100;
        Q(1, 1) = 100;
        Q(2, 2) = 2000;
        break;
      case AUTOMATIC:
        for (int i = 0; i != n; ++i) Q(i, i) = 1000;
        break;
      default:
        break;
    }
  }
  // modify penality for each error (automatic controller)
  void resetQ() {
    for (int i = 0; i != n; ++i) Q(i, i) = 1000;
  }
  //
  vectormd getlx() const { return lx; }
  vectormd getly() const { return ly; }
  vectormd getupper_delta_alpha() const { return upper_delta_alpha; }
  vectormd getlower_delta_alpha() const { return lower_delta_alpha; }
  vectormd getupper_delta_u() const { return upper_delta_u; }
  vectormd getlower_delta_u() const { return lower_delta_u; }
  matrixnnd getQ() const { return Q; }
  matrixmmd getOmega() const { return Omega; }
  matrixmmd getQ_deltau() const { return Q_deltau; }
  vectormd getg_deltau() const { return g_deltau; }
  vectormd getd_rho() const { return d_rho; }
  matrixnmd getB_alpha() const { return B_alpha; }
  matrixnmd getd_Balpha_u() const { return d_Balpha_u; }
  vectornd getb() const { return b; }
  vectormd getdelta_alpha() const { return delta_alpha; }
  vectormd getdelta_u() const { return delta_u; }
  Eigen::Matrix<double, 2 * m + n, 1> getresults() const { return results; }

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
  vectormd lx;
  vectormd ly;
  // real time constraints of each thruster
  vectormd upper_delta_alpha;
  vectormd lower_delta_alpha;
  vectormd upper_delta_u;
  vectormd lower_delta_u;
  // quadratic objective
  matrixnnd Q;
  matrixmmd Omega;
  matrixmmd Q_deltau;
  // linear objective
  vectormd g_deltau;
  vectormd d_rho;

  // real time constraint matrix in QP (equality constraint)
  matrixnmd B_alpha;
  matrixnmd d_Balpha_u;  // Jocobian matrix of Balpha times u
  vectornd b;

  // real time physical variable in thruster allocation
  vectormd delta_alpha;  // rad
  vectormd delta_u;      // N
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

  // mosek environment
  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  MSKrescodee r;

  void initializethrusterallocation(controllerRTdata<m, n> &_RTdata) {
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
    setQ(MANUAL);

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
  void calculateconstrains_tunnel(const controllerRTdata<m, n> &_RTdata,
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
          upper_delta_u(i) =
              _Kn * std::pow(_RTdata.rotation(i) - _maxdeltar, 2) -
              _RTdata.u(i);
        }

      } else if (-_maxdeltar <= _RTdata.rotation(i) &&
                 _RTdata.rotation(i) < 0) {
        if (_desired_Mz > 0) {
          // specify the second case
          upper_delta_alpha(i) = M_PI;
          lower_delta_alpha(i) = M_PI;
          lower_delta_u(i) = -_RTdata.u(i);
          upper_delta_u(i) =
              _Kp * std::pow(_RTdata.rotation(i) + _maxdeltar, 2) -
              _RTdata.u(i);
        } else {
          // specify the first case
          upper_delta_alpha(i) = 0;
          lower_delta_alpha(i) = 0;
          lower_delta_u(i) = -_RTdata.u(i);
          upper_delta_u(i) =
              _Kn * std::pow(_RTdata.rotation(i) - _maxdeltar, 2) -
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
  void calculateconstrains_azimuth(const controllerRTdata<m, n> &_RTdata) {
    for (int j = 0; j != num_azimuth; ++j) {
      int index_azimuth = j + num_tunnel;
      /* contraints on the increment of angle */
      upper_delta_alpha(index_azimuth) = std::min(
          v_azimuththrusterdata[j].max_delta_alpha,
          v_azimuththrusterdata[j].max_alpha - _RTdata.alpha(index_azimuth));
      lower_delta_alpha(index_azimuth) = std::max(
          -v_azimuththrusterdata[j].max_delta_alpha,
          v_azimuththrusterdata[j].min_alpha - _RTdata.alpha(index_azimuth));
      /* contraints on the increment of thrust */
      double K = v_azimuththrusterdata[j].K;
      int _maxdeltar = v_azimuththrusterdata[j].max_delta_rotation;
      double _thrust = K * std::pow(_RTdata.rotation(index_azimuth), 2);
      upper_delta_u(index_azimuth) =
          std::min(
              v_azimuththrusterdata[j].max_thrust,
              K * std::pow(_RTdata.rotation(index_azimuth) + _maxdeltar, 2)) -
          _thrust;

      lower_delta_u(index_azimuth) =
          std::max(
              K * std::pow(_RTdata.rotation(index_azimuth) - _maxdeltar, 2),
              v_azimuththrusterdata[j].min_thrust) -
          _thrust;
    }
  }

  // calculate vessel parameters at the next time step
  void updateNextstep(controllerRTdata<m, n> &_RTdata) {
    // calculate delta variable using Mosek results
    delta_u = results.head(m);
    delta_alpha = results.segment(m, m);
    // update alpha and u
    updateAlphaandU(_RTdata.u, _RTdata.alpha);
    // convert the double alpha(rad) to int alpha(deg)
    convertalpharadian2int(_RTdata.alpha, _RTdata.alpha_deg);
    // update rotation speed
    calculaterotation(_RTdata);
    // update BalphaU
    _RTdata.BalphaU = calculateBalphau(_RTdata.alpha, _RTdata.u);
  }

  // update alpha and u using computed delta_alpha and delta_u
  void updateAlphaandU(vectormd &_u, vectormd &_alpha) {
    _u += delta_u;
    _alpha += delta_alpha;
  }
  // convert the radian to deg, and round to integer
  void convertalpharadian2int(const vectormd &_alpha, vectormi &_alpha_deg) {
    // round to int (deg)
    for (int i = 0; i != m; ++i)
      _alpha_deg(i) = static_cast<int>(_alpha(i) * 180 / M_PI);
  }

  // calcuate rotation speed of each thruster based on thrust
  void calculaterotation(controllerRTdata<m, n> &_RTdata) {
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
  double calculateRhoTerm(const vectormd &t_alpha, double epsilon = 0.1,
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
  // calculate g_deltau and Q_deltau
  void calculateDeltauQ(const vectormd &t_u) {
    vectormd d_utemp = vectormd::Zero();
    d_utemp = t_u.cwiseSqrt();
    g_deltau = 1.5 * d_utemp;
    vectormd Q_temp = vectormd::Zero();
    Q_temp = 0.75 * d_utemp.cwiseInverse();
    Q_deltau = Q_temp.asDiagonal();
  }
  // calculate the BalphaU and b
  void calculateb(const vectornd &_tau, const vectornd &_BalphaU) {
    b = _tau - _BalphaU;
  }

  // update parameters in thruster allocation for each time step
  void updateTAparameters(const controllerRTdata<m, n> &_RTdata) {
    B_alpha = calculateBalpha(_RTdata.alpha);
    calculateJocobianRhoTerm(_RTdata.alpha);
    calculateJocobianBalphaU(_RTdata.alpha, _RTdata.u);
    calculateDeltauQ(_RTdata.u);
    calculateb(_RTdata.tau, _RTdata.BalphaU);
    calculateconstrains_tunnel(_RTdata, _RTdata.tau(2));
    calculateconstrains_azimuth(_RTdata);
  }
  // update parameters in QP for each time step
  void updateMosekparameters() {
    // update A values
    for (int i = 0; i != m; ++i) {
      for (int j = 0; j != n; ++j) {
        aval[n * i + j] = B_alpha(j, i);
        aval[n * (i + m) + j] = d_Balpha_u(j, i);
      }
    }

    // update linear constraints
    for (int i = 0; i != num_constraints; ++i) {
      blc[i] = b(i);
      buc[i] = b(i);
    }

    for (int i = 0; i != m; ++i) {
      // update variable constraints
      blx[i] = lower_delta_u(i);
      bux[i] = upper_delta_u(i);
      blx[m + i] = lower_delta_alpha(i);
      bux[m + i] = upper_delta_alpha(i);
      // update objective g and Q
      g[i] = g_deltau(i);
      g[i + m] = d_rho(i);
      qval[i] = Q_deltau(i, i);
    }
  }
  // solve QP using Mosek solver
  void onestepmosek() {
    MSKint32t i, j;
    double t_results[numvar];
    results.setZero();
    if (r == MSK_RES_OK) {
      for (j = 0; j < numvar; ++j) {
        /* Set the linear term g_j in the objective.*/
        r = MSK_putcj(task, j, g[j]);

        /* Set the bounds on variable j.
         blx[j] <= x_j <= bux[j] */
        r = MSK_putvarbound(task, j, /* Index of variable.*/
                            bkx[j],  /* Bound key.*/
                            blx[j],  /* Numerical value of lower bound.*/
                            bux[j]); /* Numerical value of upper bound.*/

        /* Input column j of A */
        r = MSK_putacol(
            task, j,             /* Variable (column) index.*/
            aptre[j] - aptrb[j], /* Number of non-zeros in column j.*/
            asub + aptrb[j],     /* Pointer to row indexes of column j.*/
            aval + aptrb[j]);    /* Pointer to Values of column j.*/
      }

      /* Set the bounds on constraints.
         for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
      for (i = 0; i < num_constraints; ++i)
        r = MSK_putconbound(task, i, /* Index of constraint.*/
                            bkc[i],  /* Bound key.*/
                            blc[i],  /* Numerical value of lower bound.*/
                            buc[i]); /* Numerical value of upper bound.*/

      /* Input the Q for the objective. */
      r = MSK_putqobj(task, numvar, qsubi, qsubj, qval);

      if (r == MSK_RES_OK) {
        MSKrescodee trmcode;

        /* Run optimizer */
        r = MSK_optimizetrm(task, &trmcode);

        /* Print a summary containing information
           about the solution for debugging purposes*/
        // MSK_solutionsummary(task, MSK_STREAM_MSG);

        if (r == MSK_RES_OK) {
          MSKsolstae solsta;
          MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

          switch (solsta) {
            case MSK_SOL_STA_OPTIMAL:
            case MSK_SOL_STA_NEAR_OPTIMAL: {
              /* Request the interior solution. */
              MSK_getxx(task, MSK_SOL_ITR, t_results);
              for (int k = 0; k != numvar; ++k) results(k) = t_results[k];
              break;
            }

            case MSK_SOL_STA_DUAL_INFEAS_CER:
            case MSK_SOL_STA_PRIM_INFEAS_CER:
            case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
            case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER: {
              CLOG(ERROR, "mosek")
                  << "Primal or dual infeasibility certificate found.";
              break;
            }
            case MSK_SOL_STA_UNKNOWN: {
              CLOG(ERROR, "mosek") << "The status of the solution could not be "
                                      "determined.";
              break;
            }
            default: {
              CLOG(ERROR, "mosek") << "Other solution status.";
              break;
            }
          }
        } else {
          CLOG(ERROR, "mosek") << "Error while optimizing.";
        }
      }
      if (r != MSK_RES_OK) {
        /* In case of an error print error code and description. */
        char symname[MSK_MAX_STR_LEN];
        char desc[MSK_MAX_STR_LEN];
        MSK_getcodedesc(r, symname, desc);

        CLOG(ERROR, "mosek")
            << "An error occurred while optimizing. " << std::string(symname)
            << " - " << std::string(desc);
      }
    }
  }
};

#endif /* _THRUSTALLOCATION_H_*/