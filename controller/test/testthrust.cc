/*
*******************************************************************************
* testthrust.cpp:
* unit test for thrust allocation
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "thrustallocation.h"
#include "utilityio.h"

INITIALIZE_EASYLOGGINGPP

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
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata);
  _thrustallocation.initializapropeller(_controllerRTdata);
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

  thrustallocationdata _thrustallocationdata{num_tunnel, num_azimuth,
                                             num_mainrudder, index_thrusters};

  std::vector<tunnelthrusterdata> v_tunnelthrusterdata;
  v_tunnelthrusterdata.reserve(num_tunnel);
  v_tunnelthrusterdata.push_back(
      {1.9, 0, 3.7e-7, 1.7e-7, 50, 3000, 3.33, 1.53});

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -1.893,            // lx
      -0.216,            // ly
      2e-5,              // K
      20,                // max_delta_rotation
      1000,              // max rotation
      10,                // min_rotation
      0.1277,            // max_delta_alpha
      M_PI * 175 / 180,  // max_alpha
      M_PI / 18,         // min_alpha
      20,                // max_thrust
      0.002              // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.893,             // lx
      0.216,              // ly
      2e-5,               // K
      20,                 // max_delta_rotation
      1000,               // max rotation
      10,                 // min_rotation
      0.1277,             // max_delta_alpha
      -M_PI / 18,         // max_alpha
      -M_PI * 175 / 180,  // min_alpha
      20,                 // max_thrust
      0.002               // min_thrust
  });

  // controllerRTdata<m, n> _controllerRTdata{
  //     (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),         // tau
  //     Eigen::Matrix<double, n, 1>::Zero(),                           //
  //     BalphaU (Eigen::Matrix<double, m, 1>() << 0.01, 0.2, 0.2).finished(),
  //     // u (Eigen::Matrix<int, m, 1>() << 0, 0, 0).finished(),            //
  //     rotation (Eigen::Matrix<double, m, 1>() << M_PI / 2, M_PI / 10, -M_PI /
  //     4)
  //         .finished(),                  // alpha
  //     Eigen::Matrix<int, m, 1>::Zero()  // alpha_deg

  // };

  controllerRTdata<m, n> _controllerRTdata{
      Eigen::Matrix<double, n, 1>::Zero(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, m, 1>::Zero(),  // u
      Eigen::Matrix<int, m, 1>::Zero(),     // rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // alpha
      Eigen::Matrix<int, m, 1>::Zero()      // alpha_deg
  };

  // initialize the thrust allocation
  thrustallocation<m, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata);
  _thrustallocation.initializapropeller(_controllerRTdata);

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
  for (int i = 0; i != 120; ++i) {
    angle = (i + 1) * M_PI / 60;
    save_tau(2, i + 1) = 0.5 * sin(angle) + 0.1 * std::rand() / RAND_MAX;
  }
  save_tau.block(1, 0, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 0.2) +
                                 0.0 * Eigen::MatrixXd::Random(1, 100);
  save_tau.block(1, 100, 1, 100) = Eigen::MatrixXd::Constant(1, 100, -0.2) +
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

  save_tau = save_tau.transpose().eval();
  save_u = save_u.transpose().eval();
  save_alpha = save_alpha.transpose().eval();
  save_alpha_deg = save_alpha_deg.transpose().eval();
  save_Balphau = save_Balphau.transpose().eval();
  save_rotation = save_rotation.transpose().eval();
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

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";
  test_multiplethrusterallocation();

  LOG(INFO) << "Shutting down.";
  return 0;
}