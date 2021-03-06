/*
*******************************************************************************
* testcontroller.cc:
* unit test for thrust allocation
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "controller.h"
#include "utilityio.h"

INITIALIZE_EASYLOGGINGPP

void test_multiplecontroller() {
  // set the parameters in the thrust allocation
  const int L = 30;
  const int m = 3;
  const int n = 3;
  constexpr ACTUATION index_actuation = FULLYACTUATED;

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
      {1.9, 0, 3.7e-7, 1.7e-7, 100, 3000, 3.33, 1.53});

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      -1.893,            // lx
      -0.216,            // ly
      2e-5,              // K
      20,                // max_delta_rotation
      1000,              // max rotation
      5,                 // min_rotation
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
      5,                  // min_rotation
      0.1277,             // max_delta_alpha
      -M_PI / 18,         // max_alpha
      -M_PI * 175 / 180,  // min_alpha
      20,                 // max_thrust
      0.05                // min_thrust
  });
  std::vector<ruddermaindata> v_ruddermaindata;

  controllerRTdata<m, n> _controllerRTdata{
      (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),         // tau
      Eigen::Matrix<double, n, 1>::Zero(),                           // BalphaU
      (Eigen::Matrix<double, m, 1>() << 0.01, 0.2, 0.2).finished(),  // u
      (Eigen::Matrix<int, m, 1>() << 0, 0, 0).finished(),            // rotation
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, M_PI / 10, -M_PI / 4)
          .finished(),                  // alpha
      Eigen::Matrix<int, m, 1>::Zero()  // alpha_deg
  };

  vessel _vessel{
      (Eigen::Matrix3d() << 100, 0, 1, 0, 100, 1, 0, 1000).finished(),  // Mass
      Eigen::Matrix3d::Zero(),  // AddedMass
      (Eigen::Matrix3d() << 100, 0, 0, 0, 200, 0, 0, 0, 300)
          .finished(),          // Damping
      Eigen::Vector2d::Zero(),  // cog
      Eigen::Vector2d::Zero(),  // x_thrust
      Eigen::Vector2d::Zero(),  // y_thrust
      Eigen::Vector2d::Zero(),  // mz_thrust
      Eigen::Vector2d::Zero(),  // surge_v
      Eigen::Vector2d::Zero(),  // sway_v
      Eigen::Vector2d::Zero(),  // yaw_v
      Eigen::Vector2d::Zero(),  // roll_v
      0,                        // L
      0                         // B
  };

  controllerdata _controllerdata{
      0.1,              // sample_time
      DYNAMICPOSITION,  // controlmode
      WINDON,           // windstatus
      index_actuation   // index_actuation
  };

  std::vector<pidcontrollerdata> v_pidcontrollerdata;
  v_pidcontrollerdata.reserve(n);
  v_pidcontrollerdata.push_back({1, 1, 1, 1, 0.1, 0.1, -1, 3});
  v_pidcontrollerdata.push_back({1, 2, 3, 1, 0.1, 0.1, -1, 3});
  v_pidcontrollerdata.push_back({1, 2, 10, 1, 0.1, 0.1, -2, 4});

  controller<L, m, index_actuation, n> _controller(
      _controllerdata, _vessel, v_pidcontrollerdata, _thrustallocationdata,
      v_tunnelthrusterdata, v_azimuththrusterdata, v_ruddermaindata);
  _controller.initializecontroller(_controllerRTdata);
  Eigen::Matrix<double, n, 1> error;
  Eigen::Matrix<double, n, 1> derror;
  Eigen::Matrix<double, n, 1> command;
  Eigen::Matrix<double, n, 1> windload;
  Eigen::Matrix<double, n, 1> v_setpoint;

  error << 1, 3, 4;
  derror << 0.9, 1, 4;
  command << 0.0, 0, 0;
  windload << 0, 0, 0;
  v_setpoint << 0, 0, 0;
  const int totalstep = 10;

  Eigen::MatrixXd save_u = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXd save_alpha = Eigen::MatrixXd::Zero(m, totalstep);
  Eigen::MatrixXi save_alpha_deg = Eigen::MatrixXi::Zero(m, totalstep);
  Eigen::MatrixXd save_Balphau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXd save_tau = Eigen::MatrixXd::Zero(n, totalstep);
  Eigen::MatrixXi save_rotation = Eigen::MatrixXi::Zero(n, totalstep);

  for (int i = 0; i != 10; ++i) {
    _controller.controlleronestep(_controllerRTdata, windload, error, derror,
                                  command, v_setpoint);

    std::cout << "step " << i << std::endl;
    std::cout << _controllerRTdata.tau << std::endl;
    std::cout << _controllerRTdata.u << std::endl;
  }
}

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);
  LOG(INFO) << "The program has started!";
  test_multiplecontroller();
  LOG(INFO) << "Shutting down.";
  return 0;
}