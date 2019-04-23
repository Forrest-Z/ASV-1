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
// #define M_PI 3.14159265358979323846

int main() {
  std::vector<int> index_thrusters{1, 1, 2, 2};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  thrustallocationdata _thrustallocationdata{num_tunnel, num_azimuth,
                                             num_mainrudder, index_thrusters};

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(num_tunnel);
  v_tunnelthrusterdata.push_back({1.9, 0, 0.01, 0.02, 1, 1000, 4, 3});
  v_tunnelthrusterdata.push_back({1, 0, 0.01, 0.02, 1, 1000, 4, 3});

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -1.893,  // lx
      -0.216,  // ly
      2e-5,    // K
      10,      // max_delta_rotation
      1000,    // max rotation
      10,      // min_rotation
      10.1,    // max_delta_alpha
      3.14,    // max_alpha
      -3.14,   // min_alpha
      100,     // max_thrust
      10       // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.893,  // lx
      0.216,   // ly
      2e-5,    // K
      10,      // max_delta_rotation
      1000,    // max rotation
      10,      // min_rotation
      10.1,    // max_delta_alpha
      -3.14,   // max_alpha
      -3.14,   // min_alpha
      100,     // max_thrust
      10       // min_thrust
  });

  controllerRTdata _controllerRTdata{
      vectornd::Zero(),                        // BalphaU
      (vectormd() << 0, 10, 0, 0).finished(),  // u
      // vectormi()::Zero(),                    // rotation
      (vectormi() << 100, 500, 400, 300).finished(),
      (vectormd() << M_PI / 2, -M_PI / 2, M_PI, 0).finished(),  // alpha
      vectormi::Zero()                                          // alpha_deg

  };

  thrustallocation<num_thrusters, num_controlspace> _thrustallocation(
      _controllerRTdata, _thrustallocationdata, v_tunnelthrusterdata,
      v_azimuththrusterdata);

  std::cout << _controllerRTdata.BalphaU << std::endl;
  std::cout << _controllerRTdata.u << std::endl;
  std::cout << _controllerRTdata.rotation << std::endl;
  std::cout << _controllerRTdata.alpha << std::endl;
  std::cout << _controllerRTdata.alpha_deg << std::endl;
}