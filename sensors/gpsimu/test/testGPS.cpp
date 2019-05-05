#include <cstdio>
#include "gps.h"

int main() {
  try {
    run();

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}
