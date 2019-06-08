/*
*******************************************************************************
* testbiling.cc:
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
    _threadloop.mainloop();
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

//碧凌: 上古神兽名。碧凌的徒弟便是上古四大神兽：青龙、白虎、朱雀、玄武。
//相传碧凌是身穿紧身碧衣，满头青丝且瞳目为碧绿之色的太极神兽，由此而生。