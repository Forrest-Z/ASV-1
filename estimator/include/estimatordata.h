/*
***********************************************************************
* estimatordata.h:
* header file to define the constant and real-time data in the
* estimators, dependent on each autonomous system.
* This header file can be read by both C and C++ compilers
*
*  by Hu.ZH (CrossOcean.ai)
***********************************************************************
*/

#ifndef _ESTIMATORDATA_H_
#define _ESTIMATORDATA_H_

#include <Eigen/Core>
#include <Eigen/Dense>
typedef Eigen::Matrix<double, 6, 6> Matrix66d;
typedef Eigen::Matrix<double, 6, 3> Matrix63d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/********************* constant ***********************************/
struct cestimator {
  sample_time;  // sample time of estimator, controller and actuator(unit:
                // second)
};
/********************* real-time ***********************************/
struct restimator {
  CTB2G;  // body   --> global
  Measurement;
  State;
  BalphaU;  // estimated thrust
};

#endif /*_ESTIMATORDATA_H_*/