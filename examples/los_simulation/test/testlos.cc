/*
*******************************************************************************
* testlossimulation.cc:
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include <cstdio>
#include "threadloop.h"
INITIALIZE_EASYLOGGINGPP

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";

  try {
    threadloop _threadloop;
    _threadloop.testthread();
    std::cout << "please input" << std::endl;
    int input = std::getchar();
    throw input;
  } catch (const std::exception& e) {
    LOG(INFO) << "Shutting down.";
    std::cerr << "ERROR: std::logic_error" << e.what() << std::endl;
  } catch (int i) {
    LOG(INFO) << " stop signal  ";
    LOG(INFO) << "Shutting down.";
    std::terminate();

  } catch (...) {
    // ensure destuctors of auto objects are called

    LOG(INFO) << "Shutting down.";
    std::cerr << "ERROR: std::logic_error\n";
    std::terminate();
  }
  return 0;
}
