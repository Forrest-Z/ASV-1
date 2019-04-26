/*
***********************************************************************
* estimatordata.h:
* header file to define the constant and real-time data in the
* motion estimators, dependent on each autonomous system.
* This header file can be read by both C and C++ compilers
*
*  by Hu.ZH (CrossOcean.ai)
***********************************************************************
*/

#ifndef _ESTIMATORDATA_H_
#define _ESTIMATORDATA_H_

#include <Eigen/Core>
#include <Eigen/Dense>

// real-time data in the state estimators
template <int n = 3>
struct estimatorRTdata {
  Eigen::Matrix3d CTB2G;  // body  --> global
  Eigen::Matrix3d CTG2B;  // global  --> body

  Eigen::Matrix<double, 2 * n, 1> Measurement;

  /********************* state *********************************************/
  // x(surge: m), y(sway: m), yaw(theta: rad), u, v, r
  // data wroten by Kalman
  Eigen::Matrix<double, 2 * n, 1> State;
  Eigen::Matrix<double, n, 1> BalphaU;  // estimated thrust

  Eigen::Matrix<double, 6, 1> motiondata_6dof;
};

/********************* constant ***********************************/
struct cestimator {
  double sample_time;  // sample time of estimator, controller and
                       // actuator(unit: second)
};

#endif /*_ESTIMATORDATA_H_*/