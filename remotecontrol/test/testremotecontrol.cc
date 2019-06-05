/*
*******************************************************************************
* testremotecontroller.cc:
* unit test for remote controller
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <chrono>
#include <cstdio>
#include <thread>
#include "remotecontrol.h"
int main() {
  recontrolRTdata _recontrolRTdata;
  try {
    remotecontrol _remotecontrol(115200);  // zone 30n

    while (1) {
      static int i = 0;
      _remotecontrol.readserialdata(_recontrolRTdata);
      std::cout << ++i << std::endl;
      std::cout << "connection:" << _remotecontrol.checkrcconnection()
                << std::endl;
      std::cout << "right_joystick_LR=    "
                << _recontrolRTdata.right_joystick_LR << std::endl;
      std::cout << "right_joystick_UD=    "
                << _recontrolRTdata.right_joystick_UD << std::endl;
      std::cout << "left_joystick_UD=    " << _recontrolRTdata.left_joystick_UD
                << std::endl;
      std::cout << "left_joystick_LR=    " << _recontrolRTdata.left_joystick_LR
                << std::endl;
      std::cout << "SA=    " << _recontrolRTdata.SA << std::endl;
      std::cout << "SB=    " << _recontrolRTdata.SB << std::endl;
      std::cout << "SC=    " << _recontrolRTdata.SC << std::endl;
      std::cout << "SD=    " << _recontrolRTdata.SD << std::endl;
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}
