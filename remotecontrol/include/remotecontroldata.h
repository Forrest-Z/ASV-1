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
  float right_joystick_LR;  // left and right
  float right_joystick_UD;  // up and down;
  float left_joystick_UD;   // up and down;
  float left_joystick_LR;   // left and right
  float SA;                 // SA
  float SB;                 // SB
  float SC;                 // SC
  float SD;                 // SD
};

#endif /* _REMOTECONTROLDATA_H_ */