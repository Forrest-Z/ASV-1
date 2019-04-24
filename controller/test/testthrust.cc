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

void testonestepthrustallocation() {}

// void test_multiplethrusterallocation(savefile &_savefile,
//                                      const vessel_third &_vessel,
//                                      realtimevessel_third &_realtimevessel) {
//   // data saved for validation and viewer
//   const int totalstep = 200;
//   _savefile.save_u = Eigen::MatrixXd::Zero(_vessel.m, totalstep);
//   _savefile.save_alpha = Eigen::MatrixXd::Zero(_vessel.m, totalstep);
//   _savefile.save_alpha_deg = Eigen::MatrixXi::Zero(_vessel.m, totalstep);
//   _savefile.save_Balphau = Eigen::MatrixXd::Zero(_vessel.n, totalstep);
//   _savefile.save_tau = Eigen::MatrixXd::Zero(_vessel.n, totalstep);
//   _savefile.save_rotation = Eigen::MatrixXi::Zero(_vessel.n, totalstep);

//   // desired forces
//   double angle = 0;
//   for (int i = 0; i != 30; ++i) {
//     angle = (i + 1) * M_PI / 15;
//     _savefile.save_tau(2, i + 70) =
//         0.0 * sin(angle) + 0.1 * std::rand() / RAND_MAX;
//   }
//   _savefile.save_tau.block(1, 0, 1, 100) =
//       Eigen::MatrixXd::Constant(1, 100, 4) +
//       0.0 * Eigen::MatrixXd::Random(1, 100);
//   _savefile.save_tau.block(1, 100, 1, 100) =
//       Eigen::MatrixXd::Constant(1, 100, -4) +
//       0.0 * Eigen::MatrixXd::Random(1, 100);
//   _savefile.save_tau.row(0) = 0.01 * Eigen::MatrixXd::Random(1, totalstep);
//   for (int i = 0; i != totalstep; ++i) {
//     // update tau
//     _realtimevessel.tau = _savefile.save_tau.col(i);
//     // thruster allocation
//     onestepthrusterallocation(_realtimevessel);
//     // save variables
//     _savefile.save_u.col(i) = _realtimevessel.u;
//     _savefile.save_alpha.col(i) = _realtimevessel.alpha;
//     _savefile.save_alpha_deg.col(i) = _realtimevessel.alpha_deg;
//     _savefile.save_Balphau.col(i) = _realtimevessel.BalphaU;
//     _savefile.save_rotation.col(i) = _realtimevessel.rotation;
//   }
// }

int main() {
  std::vector<int> index_thrusters{1, 1, 2, 2};

  int num_tunnel =
      std::count(index_thrusters.begin(), index_thrusters.end(), 1);

  int num_azimuth =
      std::count(index_thrusters.begin(), index_thrusters.end(), 2);

  int num_mainrudder =
      std::count(index_thrusters.begin(), index_thrusters.end(), 3);

  thrustallocationdata _thrustallocationdata{
      num_tunnel, num_azimuth, num_mainrudder, index_thrusters, "logfile.txt"};

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

  // std::cout << _RTdata.alpha << std::endl;
  // std::cout << "upper_delta_alpha: " << upper_delta_alpha << std::endl;
  // std::cout << "lower_delta_alpha: " << lower_delta_alpha << std::endl;
  // std::cout << "upper_delta_u: " << upper_delta_u << std::endl;
  // std::cout << "lower_delta_u: " << lower_delta_u << std::endl;
  // std::cout << "Q: " << Q << std::endl;
  // std::cout << "Omega: " << Omega << std::endl;
  // std::cout << "Q_deltau: " << Q_deltau << std::endl;
  // std::cout << "g_deltau: " << g_deltau << std::endl;
  // std::cout << "d_rho: " << d_rho << std::endl;
  // std::cout << "B_alpha: " << B_alpha << std::endl;
  // std::cout << "d_Balpha_u: " << d_Balpha_u << std::endl;
  // std::cout << "b: " << b << std::endl;
}