/*
*******************************************************************************
* testthrust.cpp:
* function for control allocation based on Quadratic programming, using
* Mosek solver API. Normally, thrust alloation is used in the fully-actuated
* control system
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "controllerdata.h"
#include "thrustallocation.h"

int main() {
  int m = 3;
  int n = 3;
  thrustallocationdata _thrustallocationdata{m, n, Eigen::VectorXd::Zero(m)};

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(1);
  v_tunnelthrusterdata.push_back({1, 1, 1, 1, 1, 1, 0, 1});
  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(m - 1);

  thrustallocation _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata);
}