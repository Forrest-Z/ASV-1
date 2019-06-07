/*
*****************************************************************************
* testtimer.cc:
* unit test for timer in milliseconds
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*****************************************************************************
*/

#include <chrono>
#include <iostream>
#include <thread>
#include "timecounter.h"
int main() {
  timecounter _timer;
  for (int i = 0; i != 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    long int et = _timer.timeelapsed();
    std::cout << et << std::endl;
  }
}
