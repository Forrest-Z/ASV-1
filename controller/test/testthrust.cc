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
  v_tunnelthrusterdata.push_back({1.9, 0, 1e-6, 2e-6, 50, 1000, 1, 2});
  v_tunnelthrusterdata.push_back({1, 0, 1e-6, 2e-6, 50, 1000, 1, 2});

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -1.893,         // lx
      -0.216,         // ly
      2e-5,           // K
      10,             // max_delta_rotation
      1000,           // max rotation
      10,             // min_rotation
      0.1277,         // max_delta_alpha
      M_PI / 6,       // max_alpha
      -7 * M_PI / 6,  // min_alpha
      20,             // max_thrust
      2e-3            // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.893,        // lx
      0.216,         // ly
      2e-5,          // K
      10,            // max_delta_rotation
      1000,          // max rotation
      10,            // min_rotation
      0.1277,        // max_delta_alpha
      7 * M_PI / 6,  // max_alpha
      -M_PI / 6,     // min_alpha
      20,            // max_thrust
      2e-3           // min_thrust
  });

  controllerRTdata _controllerRTdata{
      (vectornd() << 0, 0, 1).finished(),       // tau
      vectornd::Zero(),                         // BalphaU
      (vectormd() << 0, 0.5, 0, 1).finished(),  // u
      // vectormi()::Zero(),                    // rotation
      (vectormi() << 100, 500, 400, 300).finished(),
      (vectormd() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3, 0).finished(),  // alpha
      vectormi::Zero()  // alpha_deg

  };

  thrustallocation<num_thrusters, num_controlspace> _thrustallocation(
      _controllerRTdata, _thrustallocationdata, v_tunnelthrusterdata,
      v_azimuththrusterdata);

  _thrustallocation.testf(_controllerRTdata);
}