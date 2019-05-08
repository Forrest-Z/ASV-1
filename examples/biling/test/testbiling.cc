/*
*******************************************************************************
* testbiling.cc:
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "threadloop.h"

INITIALIZE_EASYLOGGINGPP

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  threadloop _threadloop;
  _threadloop.testthread();

  LOG(INFO) << "Shutting down.";
  return 0;
}