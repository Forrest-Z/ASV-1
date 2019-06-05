/*
***********************************************************************
* priority.h: indicator for controller, estimator, planner, joystick
* and GUI
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _PRIORITY_H_
#define _PRIORITY_H_

// #include "controllerdata.h"
// #include "estimatordata.h"
// #include "gpsdata.h"
// #include "motorclientdata.h"
// #include "plannerdata.h"

struct indicators {
  // indicator for gui connection: 0 --> disconnect, 1 -->connect
  int gui_connection;
  // indicator for joystick connection: 0 --> disconnect, 1 -->connect
  int joystick_connection;
  int indicator_controlmode;
  int indicator_windstatus;
};

#endif /* _PRIORITY_H_ */
