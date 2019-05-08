/*
***********************************************************************
* testlog.h:
* unit test for easylogging, with multiple loggers to the same file
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _TESTLOG_H_
#define _TESTLOG_H_
#include "easylogging++.h"
class testlog {
 public:
  testlog() {}
  ~testlog() {}

  void def() {
    for (int i = 0; i < 1000; ++i) {
      CLOG(INFO, "first") << "This is from first " << i;
    }
    CLOG(ERROR, "first") << "This is info log using performance logger";
  }
  void second() {
    for (int i = 0; i < 1000; ++i)
      CLOG(INFO, "second") << "This is from second" << i;
  }

 private:
};
#endif /*_TESTLOG_H_*/