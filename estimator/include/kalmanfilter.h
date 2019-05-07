/*
***********************************************************************
* kalmanfilter.h: observer using kalman filter
* function to simulate the kalman filter based on Eigen
* This header file can be read by C++ compilers
*
*  Kalman Filter Class Definition.
*
*  Matrix Dimension must be:
*  x[k] = A * x[k-1] + B * u[k-1] + w[k-1]
*  z[k] = H * x[k] + v[k]
*  x: n x 1, state vector
*  z: m x 1, observer vector
*  u: l x 1, input vector
*  A: n x n
*  B: n x l
*  H: m x n
*  Q: n x n
*  R: m x m
*  I: n x n
*  P: n x n
*  K: n x n
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "estimatordata.h"
#include "vesseldata.h"

// Kalman filtering for linear system
class kalmanfilter {
 public:
  // disable the default constructor
  kalmanfilter() = delete;
  explicit kalmanfilter(const Eigen::MatrixXd &_A, const Eigen::MatrixXd &_B,
                        const Eigen::MatrixXd &_H, const Eigen::MatrixXd &_Q,
                        const Eigen::MatrixXd &_R) noexcept
      : A(_A),
        B(_B),
        H(_H),
        Q(_Q),
        R(_R),
        n(_A.rows()),
        l(_B.cols()),
        m(_H.rows()),
        In(n, n),
        Im(m, m) {
    In.setIdentity();
    Im.setIdentity();
  }

  ~kalmanfilter() noexcept {}

  /* Set Initial Value */
  void setInitial(const Eigen::VectorXd &_X0, const Eigen::MatrixXd &_P0) {
    X0 = _X0;
    P0 = _P0;
  }

  // perform kalman filter for one step
  kalmanfilter &kalmanonestep(const Eigen::MatrixXd &_A,
                              const Eigen::MatrixXd &_B,
                              const Eigen::VectorXd &former_U,
                              const Eigen::VectorXd &_Z) {
    updatesystem(_A, _B);
    predict(former_U);
    correct(_Z);
    return *this;
  }

  kalmanfilter &kalmanonestep(const Eigen::MatrixXd &_A,
                              const Eigen::VectorXd &former_U,
                              const Eigen::VectorXd &_Z) {
    updatesystem(_A);
    predict(former_U);
    correct(_Z);
    return *this;
  }
  kalmanfilter &kalmanonestep(const Eigen::VectorXd &former_U,
                              const Eigen::VectorXd &_Z) {
    predict(former_U);
    correct(_Z);
    return *this;
  }

  Eigen::VectorXd getState() const noexcept { return X; }

  // calculate the max eigenvalue of P
  double getMaxEigenP() const {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(P0);
    if (eigensolver.info() != Eigen::Success)
      return 100;
    else
      return eigensolver.eigenvalues().maxCoeff();
  }

 private:
  /* Fixed Matrix */
  Eigen::MatrixXd A;  // System dynamics matrix
  Eigen::MatrixXd B;  // Control matrix
  Eigen::MatrixXd H;  // Mesaurement Adaptation matrix
  Eigen::MatrixXd Q;  // Process Noise Covariance matrix
  Eigen::MatrixXd R;  // Measurement Noise Covariance matrix

  /* Problem Dimension */
  int n;  // State vector dimension
  int l;  // Control vector (input) dimension (if there is not input, set to
          // zero)
  int m;  // observer vector
  Eigen::MatrixXd In;  // Identity matrix
  Eigen::MatrixXd Im;  // Identity matrix
  /* Variable Matrix */
  Eigen::VectorXd X;  //(Current) State vector
  Eigen::MatrixXd P;  // State Covariance
  Eigen::MatrixXd K;  // Kalman Gain matrix

  /* Inizial Value */
  Eigen::VectorXd X0;  // Initial State vector
  Eigen::MatrixXd P0;  // Initial State Covariance matrix

  /* Do prediction based of physical system (No external input) */
  void predict(void) {
    X = A * X0;
    P = A * P0 * A.transpose() + Q;
  }

  /* Do prediction based of physical system (with external input)
   * U: Control vector
   */
  void predict(const Eigen::VectorXd &U) {
    X = A * X0 + B * U;
    P = A * P0 * A.transpose() + Q;
  }

  /* Correct the prediction, using mesaurement
   *  Z: mesaure vector
   */
  void correct(const Eigen::VectorXd &Z) {
    // K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
    K = (P * H.transpose()) * (H * P * H.transpose() + R).llt().solve(Im);
    X = X + K * (Z - H * X);

    P = (In - K * H) * P;

    X0 = X;
    P0 = P;
  }

  /*Set Fixed Matrix(NO INPUT) */
  void updatesystem(const Eigen::MatrixXd &_A, const Eigen::MatrixXd &_B) {
    A = _A;
    B = _B;
  }
  /*Set Fixed Matrix(NO INPUT) */
  void updatesystem(const Eigen::MatrixXd &_A) { A = _A; }
};

// Kalman filtering for surface vessel
template <int l = 3, int m = 6, int n = 6>
class kalmanfilterv {
  using vectorld = Eigen::Matrix<double, l, 1>;
  using vectormd = Eigen::Matrix<double, m, 1>;
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixnnd = Eigen::Matrix<double, n, n>;
  using matrixnld = Eigen::Matrix<double, n, l>;
  using matrixmnd = Eigen::Matrix<double, m, n>;
  using matrixmmd = Eigen::Matrix<double, m, m>;

 public:
  // disable the default constructor
  kalmanfilterv() = delete;
  explicit kalmanfilterv(const vessel &_vessel, double _sample_time) noexcept
      : A(matrixnnd::Zero()),
        B(matrixnld::Zero()),
        H(matrixmnd::Identity()),
        Q(matrixnnd::Identity()),
        R(matrixmmd::Identity()),
        P(matrixnnd::Identity()),
        K(matrixnnd::Zero()),
        X(vectornd::Zero()),
        sample_time(_sample_time) {
    initializekalman(_vessel);
  }

  ~kalmanfilterv() noexcept {}

  // perform kalman filter for one step
  kalmanfilterv &kalmanonestep(const estimatorRTdata &_RTdata) {
    updateKalmanA(_RTdata.CTB2G);
    predict(_RTdata.BalphaU);
    correct(_RTdata.Measurement);
    return *this;
  }

  // After intialization of sensors, we can specify value to state
  void setState(const vectornd &_state) { X = _state; }
  vectornd getState() const noexcept { return X; }

  // calculate the max eigenvalue of P
  double getMaxEigenP() const {
    Eigen::SelfAdjointEigenSolver<matrixnnd> eigensolver(P);
    if (eigensolver.info() != Eigen::Success)
      return 100;
    else
      return eigensolver.eigenvalues().maxCoeff();
  }

  //
  void setQ(const matrixnnd &_Q) { Q = _Q; }
  void setR(const matrixmmd &_R) { R = _R; }

 private:
  /* Fixed Matrix */
  matrixnnd A;  // System dynamics matrix
  matrixnld B;  // Control matrix
  matrixmnd H;  // Mesaurement Adaptation matrix
  matrixnnd Q;  // Process Noise Covariance matrix
  matrixmmd R;  // Measurement Noise Covariance matrix

  /* Variable Matrix */
  matrixnnd P;  // State Covariance
  matrixnnd K;  // Kalman Gain matrix
  vectornd X;   //(Current) State vector

  const double sample_time;

  // initialize parameters in Kalman filter
  void initializekalman(const vessel &_vessel) {
    // copy the constant data
    Eigen::Matrix3d Mass(_vessel.Mass + _vessel.AddedMass);
    Eigen::Matrix3d Damping(_vessel.Damping);

    // calcualte the A and B in continous equation
    matrixnld Bk = matrixnld::Zero();
    matrixnnd Ak = matrixnnd::Zero();
    Eigen::Matrix3d Inv_Mass = Mass.inverse();
    Ak.topRightCorner(3, 3) = Eigen::Matrix3d::Identity();
    Ak.bottomRightCorner(3, 3) = -Inv_Mass * Damping;
    Bk.bottomRows(3) = Inv_Mass;

    // calculate discrete time A, B, and H
    A = matrixnnd::Identity() + sample_time * Ak;
    B = sample_time * Bk;
  }

  // real time update the Kalman filter matrix using orientation
  void updateKalmanA(const Eigen::Matrix3d &_CTB2G) {
    A.topRightCorner(3, 3) = sample_time * _CTB2G;
  }

  void predict(const vectorld &_U) {
    X = A * X + B * _U;
    P = A * P * A.transpose() + Q;
  }

  /* Correct the prediction, using mesaurement */
  void correct(const vectormd &_measurement) {
    K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
    // K = (P * H.transpose()) * (H * P * H.transpose() + R).llt().solve(Im);
    X = X + K * (_measurement - H * X);
    P = (matrixnnd::Identity() - K * H) * P;
  }
};

#endif /* _KALMANFILTER_H_ */