#include "guiclient.h"

int main() {
  guiclient _guiclient;
  timecounter _timer;
  while (1) {
    _guiclient.guicommunication();
    std::cout << _timer.timeelapsed() << std::endl;
  }
}