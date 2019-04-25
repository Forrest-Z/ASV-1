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
#include "utilityio.h"

void testonestepthrustallocation() {
  const int m = 4;
  const int n = 3;
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

  controllerRTdata<m, n> _controllerRTdata{
      (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),       // tau
      Eigen::Matrix<double, n, 1>::Zero(),                         // BalphaU
      (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0, 1).finished(),  // u
      // vectormi()::Zero(),                    // rotation
      (Eigen::Matrix<int, m, 1>() << 100, 500, 400, 300).finished(),
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3, 0)
          .finished(),                  // alpha
      Eigen::Matrix<int, m, 1>::Zero()  // alpha_deg

  };

  thrustallocation<m, n> _thrustallocation(
      _controllerRTdata, _thrustallocationdata, v_tunnelthrusterdata,
      v_azimuththrusterdata);

  utilityio _utilityio;

  std::cout << _controllerRTdata.alpha << std::endl;
  std::cout << "upper_delta_alpha: " << _thrustallocation.getupper_delta_alpha()
            << std::endl;
  std::cout << "lower_delta_alpha: " << _thrustallocation.getlower_delta_alpha()
            << std::endl;
  std::cout << "upper_delta_u: " << _thrustallocation.getupper_delta_u()
            << std::endl;
  std::cout << "lower_delta_u: " << _thrustallocation.getlower_delta_u()
            << std::endl;
  std::cout << "Q: " << _thrustallocation.getQ() << std::endl;
  std::cout << "Omega: " << _thrustallocation.getOmega() << std::endl;
  std::cout << "Q_deltau: " << _thrustallocation.getQ_deltau() << std::endl;
  std::cout << "g_deltau: " << _thrustallocation.getg_deltau() << std::endl;
  std::cout << "d_rho: " << _thrustallocation.getd_rho() << std::endl;
  std::cout << "B_alpha: " << _thrustallocation.getB_alpha() << std::endl;
  std::cout << "d_Balpha_u: " << _thrustallocation.getd_Balpha_u() << std::endl;
  std::cout << "lx: " << _thrustallocation.getlx() << std::endl;

  Eigen::MatrixXd vvv(2, 3);
  vvv.setZero();
  _utilityio.write2csvfile("csvfile.csv", vvv);
}

void test_multiplethrusterallocation() {
  // set the parameters in the thrust allocation
  const int m = 3;
  const int n = 3;

  std::vector<int> index_thrusters{1, 2, 2};

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
  v_tunnelthrusterdata.push_back(
      {1.9, 0, 3.7e-7, 1.7e-7, 100, 3000, 3.33, 1.53});

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -1.893,            // lx
      -0.216,            // ly
      2e-5,              // K
      20,                // max_delta_rotation
      1000,              // max rotation
      50,                // min_rotation
      0.1277,            // max_delta_alpha
      M_PI * 175 / 180,  // max_alpha
      M_PI / 18,         // min_alpha
      20,                // max_thrust
      0.05               // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.893,             // lx
      0.216,              // ly
      2e-5,               // K
      20,                 // max_delta_rotation
      1000,               // max rotation
      50,                 // min_rotation
      0.1277,             // max_delta_alpha
      -M_PI / 18,         // max_alpha
      -M_PI * 175 / 180,  // min_alpha
      20,                 // max_thrust
      0.05                // min_thrust
  });

  controllerRTdata<m, n> _controllerRTdata{
      (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),         // tau
      Eigen::Matrix<double, n, 1>::Zero(),                           // BalphaU
      (Eigen::Matrix<double, m, 1>() << 0.01, 0.2, 0.2).finished(),  // u
      (Eigen::Matrix<int, m, 1>() << 0, 0, 0).finished(),            // rotation
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, M_PI / 10, -M_PI / 4)
          .finished(),                  // alpha
      Eigen::Matrix<int, m, 1>::Zero()  // alpha_deg

  };

  // initialize the thrust allocation
  thrustallocation<m, n> _thrustallocation(
      _controllerRTdata, _thrustallocationdata, v_tunnelthrusterdata,
      v_azimuththrusterdata);

  // data saved for validation and viewer
  const int totalstep = 200;

  Eigen::MatrixXd save_u = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXd save_alpha = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXi save_alpha_deg = Eigen::MatrixXi::Zero(m, totalstep);
  Eigen::MatrixXd save_Balphau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXd save_tau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXi save_rotation = Eigen::MatrixXi::Zero(n, totalstep);

  // desired forces
  double angle = 0;
  for (int i = 0; i != 30; ++i) {
    angle = (i + 1) * M_PI / 15;
    save_tau(2, i + 70) = 0.0 * sin(angle) + 0.1 * std::rand() / RAND_MAX;
  }
  save_tau.block(1, 0, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 4) +
                                 0.0 * Eigen::MatrixXd::Random(1, 100);
  save_tau.block(1, 100, 1, 100) = Eigen::MatrixXd::Constant(1, 100, -4) +
                                   0.0 * Eigen::MatrixXd::Random(1, 100);
  save_tau.row(0) = 0.01 * Eigen::MatrixXd::Random(1, totalstep);
  for (int i = 0; i != totalstep; ++i) {
    // update tau
    _controllerRTdata.tau = save_tau.col(i);
    // thruster allocation
    _thrustallocation.onestepthrustallocation(_controllerRTdata);
    // save variables
    save_u.col(i) = _controllerRTdata.u;
    save_alpha.col(i) = _controllerRTdata.alpha;
    save_alpha_deg.col(i) = _controllerRTdata.alpha_deg;
    save_Balphau.col(i) = _controllerRTdata.BalphaU;
    save_rotation.col(i) = _controllerRTdata.rotation;
  }

  // save data to csv file
  utilityio _utilityio;
  std::string _name("../data/");
  _utilityio.write2csvfile(_name + "tau.csv", save_tau);
  _utilityio.write2csvfile(_name + "u.csv", save_u);
  _utilityio.write2csvfile(_name + "alpha.csv", save_alpha);
  _utilityio.write2csvfile(_name + "alpha_deg.csv", save_alpha_deg);
  _utilityio.write2csvfile(_name + "Balpha.csv", save_Balphau);
  _utilityio.write2csvfile(_name + "rotation.csv", save_rotation);
}

int main() { test_multiplethrusterallocation(); }