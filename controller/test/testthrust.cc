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
  std::vector<int> index_thrusters{1, 2, 2};

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
  v_tunnelthrusterdata.push_back({1, 1, 1, 1, 1, 1, 0, 1});
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

  thrustallocation<num_thrusters, num_controlspace> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata);
}