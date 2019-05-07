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
struct estimatorRTdata {
  Eigen::Matrix3d CTB2G;  // body  --> global
  Eigen::Matrix3d CTG2B;  // global  --> body

  /********************* measured data  *********************/
  // x(m), y(m), orientation(theta: rad), u, v, r (next time stamp)
  // data wroten by sensors
  Eigen::Matrix<double, 6, 1> Measurement;

  /********************* state *********************************************/
  // x(surge: m), y(sway: m), yaw(theta: rad), u, v, r
  // data wroten by Kalman
  Eigen::Matrix<double, 6, 1> State;
  /********************* error *********************************************/
  Eigen::Matrix<double, 3, 1> p_error;  // error in surge, sway and heading
  Eigen::Matrix<double, 3, 1>
      v_error;  // velocity error in surge, sway and heading

  Eigen::Matrix<double, 3, 1> BalphaU;  // estimated thrust

  Eigen::Matrix<double, 6, 1> motiondata_6dof;
};

/********************* constant ***********************************/
struct estimatordata {
  double sample_time;  // sample time of estimator((unit: second)),
  bool kalman_use;     //
};

#endif /*_ESTIMATORDATA_H_*/