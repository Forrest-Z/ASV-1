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
  int right_joystick_LR;  // left and right
  int right_joystick_UD;  // up and down;
  int left_joystick_UD;   // up and down;
  int left_joystick_LR;   // left and right
  int SA;                 // ppm5
  int SB;                 // ppm6
  int SC;                 // ppm7
  int SD;                 // ppm8
};

#endif /* _REMOTECONTROLDATA_H_ */