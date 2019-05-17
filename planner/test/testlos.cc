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
  _lineofsight.setpos_p0(wp1);
  _lineofsight.setpos_p1(wp2);
  _lineofsight.testlos(-0.2, 0);
}