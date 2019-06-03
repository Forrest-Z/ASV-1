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
      _remotecontrol.rconestep(_recontrolRTdata);
      std::cout << ++i << std::endl;
      std::cout << "ppm1=    " << _recontrolRTdata.ppm1 << std::endl;
      std::cout << "ppm2=    " << _recontrolRTdata.ppm2 << std::endl;
      std::cout << "ppm3=    " << _recontrolRTdata.ppm3 << std::endl;
      std::cout << "ppm4=    " << _recontrolRTdata.ppm4 << std::endl;
      std::cout << "ppm5=    " << _recontrolRTdata.ppm5 << std::endl;
      std::cout << "ppm6=    " << _recontrolRTdata.ppm6 << std::endl;
      std::cout << "ppm7=    " << _recontrolRTdata.ppm7 << std::endl;
      std::cout << "ppm8=    " << _recontrolRTdata.ppm8 << std::endl;
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}
