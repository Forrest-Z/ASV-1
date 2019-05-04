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

template <int l = 3, int m = 6, int n = 6>
class kalmanfilter {
  using vectorld = Eigen::Matrix<double, l, 1>;
  using vectormd = Eigen::Matrix<double, m, 1>;
  using vectornd = Eigen::Matrix<double, n, 1>;
  using matrixnnd = Eigen::Matrix<double, n, n>;
  using matrixnld = Eigen::Matrix<double, n, l>;
  using matrixmnd = Eigen::Matrix<double, m, n>;
  using matrixmmd = Eigen::Matrix<double, m, m>;

 public:
  // disable the default constructor
  kalmanfilter() = delete;
  explicit kalmanfilter(const vessel &_vessel,
                        const estimatordata &_estimatordata) noexcept
      : A(matrixnnd::Zero()),
        B(matrixnld::Zero()),
        H(matrixmnd::Identity()),
        Q(matrixnnd::Identity()),
        R(matrixmmd::Identity()),
        P(matrixnnd::Identity()),
        K(matrixnnd::Zero()),
        X(vectornd::Zero()),
        sample_time(_estimatordata.sample_time) {
    initializekalman(_vessel);
  }

  ~kalmanfilter() noexcept {}

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
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(P);
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