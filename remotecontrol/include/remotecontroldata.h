/*
***********************************************************************
* remotecontroldata.h:
* header file to define the data for remote control
* This header file can be read by both C and C++ compilers
*
*  by Hu.ZH (CrossOcean.ai)
***********************************************************************
*/

#ifndef _REMOTECONTROLDATA_H_
#define _REMOTECONTROLDATA_H_

#include <Eigen/Core>
#include <Eigen/Dense>

// real-time data from remote controller
struct recontrolRTdata {
  int ppm1;  // ppm1
  int ppm2;  // ppm2
  int ppm3;  // ppm3
  int ppm4;  // ppm4
  int ppm5;  // ppm5
  int ppm6;  // ppm6
  int ppm7;  // ppm7
  int ppm8;  // ppm8
};

#endif /* _REMOTECONTROLDATA_H_ */