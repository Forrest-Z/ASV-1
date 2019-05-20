/*
*******************************************************************************
* testlos.cc:
* unit test for path following using line of sight algorithm
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "lineofsight.h"

int main() {
  double L = 1.255;
  Eigen::Vector2d wp1 = Eigen::Vector2d::Zero();
  Eigen::Vector2d wp2 = Eigen::Vector2d::Zero();
  wp1 << 0.372, -0.181;
  wp2 << -0.628, 1.320;
  lineofsight _lineofsight(L, 0);

  Eigen::Vector2d vp = Eigen::Vector2d::Zero();
  vp << -0.2, 0;
  std::cout << _lineofsight.computelospoint(vp, wp1, wp2) << std::endl;
}