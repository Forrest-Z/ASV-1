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

/********************* constant ***********************************/
struct cestimator {
  double sample_time;  // sample time of estimator, controller and
                       // actuator(unit: second)
};
/********************* real-time ***********************************/
struct restimator {
  Eigen::MatrixXd CTB2G;  // body   --> global
  Eigen::VectorXd Measurement;
  Eigen::VectorXd State;
  Eigen::VectorXd BalphaU;  // estimated thrust
};

#endif /*_ESTIMATORDATA_H_*/