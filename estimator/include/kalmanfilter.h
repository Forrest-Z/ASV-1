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
*  for Dynamic positioning, n=6, m=6, l=3
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

#include <fstream>
#include <iostream>
#include "estimatordata.h"

class kalmanfilter {
 public:
  explicit kalmanfilter(const vessel_first &_vessel) {
    initializekalman(_vessel);
  }
  kalmanfilter() = delete;
  ~kalmanfilter() {}

  // perform kalman filter for one step
  void kalmanonestep(realtimevessel_first &_realtimedata) {
    updateKalmanA(_realtimedata);
    predict(_realtimedata);
    correct(_realtimedata);
  }

  Matrix66d getA() { return A; }
  Matrix63d getB() { return B; }

 private:
  /* Fixed Matrix */
  Matrix66d A;  // System dynamics matrix
  Matrix63d B;  // Control matrix
  Matrix66d H;  // Mesaurement Adaptation matrix
  Matrix66d Q;  // Process Noise Covariance matrix
  Matrix66d R;  // Measurement Noise Covariance matrix

  /* Variable Matrix */
  Matrix66d P;  // State Covariance
  Matrix66d K;  // Kalman Gain matrix

  /* Do prediction based of physical system (with external input)
   * U: Control vector
   */
  void predict(realtimevessel_first &_realtimedata) {
    _realtimedata.State = A * _realtimedata.State + B * _realtimedata.BalphaU;
    P = A * P * A.transpose() + Q;
  }

  /* Correct the prediction, using mesaurement */
  void correct(realtimevessel_first &_realtimedata) {
    K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();
    // K = (P * H.transpose()) * (H * P * H.transpose() + R).llt().solve(Im);
    _realtimedata.State = _realtimedata.State + K * (_realtimedata.Measurement -
                                                     H * _realtimedata.State);

    P = (Matrix66d::Identity() - K * H) * P;
  }

  // initialize parameters in Kalman filter
  void initializekalman(const vessel_first &_vessel) {
    // copy the constant data
    Eigen::Matrix3d Mass(_vessel.Mass);
    Eigen::Matrix3d Damping(_vessel.Damping);

    // calcualte the A and B in continous equation
    Matrix63d Bk = Matrix63d::Zero();
    Matrix66d Ak = Matrix66d::Zero();
    Eigen::Matrix3d Inv_Mass = Mass.inverse();
    Ak.topRightCorner(3, 3) = Eigen::Matrix3d::Identity();
    Ak.bottomRightCorner(3, 3) = -Inv_Mass * Damping;
    Bk.bottomRows(3) = Inv_Mass;

    // calculate discrete time A, B, and H
    A = Matrix66d::Identity() + sample_time * Ak;
    B = sample_time * Bk;
    H = Matrix66d::Identity();
    // specify Q, R, P
    Q = 0.01 * Matrix66d::Identity();
    R = 0.1 * Matrix66d::Identity();
    P = 1 * Matrix66d::Identity();
  }

  // real time update the Kalman filter matrix using orientation
  void updateKalmanA(const realtimevessel_first &_realtimedata) {
    A.topRightCorner(3, 3) = sample_time * _realtimedata.CTB2G;
  }
};

#endif /* _KALMANFILTER_H_ */