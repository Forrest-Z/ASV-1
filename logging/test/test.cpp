//
// This file is part of Easylogging++ samples
//
// Revision 1.0
//

#include <thread>
#include "testlog.h"

INITIALIZE_EASYLOGGINGPP

void def() {
  testlog _testlog;
  _testlog.def();
}

void second() {
  testlog _testlog;
  _testlog.second();
}

int main(int, char**) {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  // el::Configurations confFromFile(
  //     "/home/scar1et/Coding/ASV/logging/default-logger.conf");

  // el::Loggers::setDefaultConfigurations(confFromFile, true);

  LOG(INFO) << "The program has started!";

  std::thread t1(def);
  std::thread t2(second);

  t1.join();
  t2.join();

  LOG(INFO) << "Shutting down.";
  return 0;
}
