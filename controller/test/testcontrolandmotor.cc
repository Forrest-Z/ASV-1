/*
*******************************************************************************
* testcontrolandmotor.cc:
* unit test for thrust allocation
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "motorclient.h"
#include "thrustallocation.h"

INITIALIZE_EASYLOGGINGPP

void test() {
  // set the parameters in the thrust allocation
  const int m = 6;
  const int n = 3;
  constexpr ACTUATION index_actuation = FULLYACTUATED;

  std::vector<int> index_thrusters{2, 2, 2, 2, 2, 2};

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

  std::vector<azimuththrusterdata> v_azimuththrusterdata;
  v_azimuththrusterdata.reserve(num_azimuth);
  v_azimuththrusterdata.push_back({
      2,           // lx
      0,           // ly
      2.8E-6,      // K
      20,          // max_delta_rotation
      1500,        // max rotation
      5,           // min_rotation
      0.1277,      // max_delta_alpha
      1.5 * M_PI,  // max_alpha
      0.5 * M_PI,  // min_alpha
      20,          // max_thrust
      0.05         // min_thrust
  });
  v_azimuththrusterdata.push_back({
      0.63,    // lx
      -0.83,   // ly
      2.8e-6,  // K
      20,      // max_delta_rotation
      1500,    // max rotation
      5,       // min_rotation
      0.1277,  // max_delta_alpha
      M_PI,    // max_alpha
      0,       // min_alpha
      20,      // max_thrust
      0.05     // min_thrust
  });
  v_azimuththrusterdata.push_back({
      0.63,    // lx
      0.83,    // ly
      2.8e-6,  // K
      20,      // max_delta_rotation
      1500,    // max rotation
      5,       // min_rotation
      0.1277,  // max_delta_alpha
      0,       // max_alpha
      -M_PI,   // min_alpha
      20,      // max_thrust
      0.05     // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -0.64,   // lx
      -0.83,   // ly
      2.8E-6,  // K
      20,      // max_delta_rotation
      1500,    // max rotation
      5,       // min_rotation
      0.1277,  // max_delta_alpha
      M_PI,    // max_alpha
      0,       // min_alpha
      20,      // max_thrust
      0.05     // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -0.64,   // lx
      0.83,    // ly
      2.8E-6,  // K
      20,      // max_delta_rotation
      1500,    // max rotation
      5,       // min_rotation
      0.1277,  // max_delta_alpha
      0,       // max_alpha
      -M_PI,   // min_alpha
      20,      // max_thrust
      0.05     // min_thrust
  });
  v_azimuththrusterdata.push_back({
      -1.72,        // lx
      0,            // ly
      2.8E-6,       // K
      20,           // max_delta_rotation
      1500,         // max rotation
      5,            // min_rotation
      0.1277,       // max_delta_alpha
      0.5 * M_PI,   // max_alpha
      -0.5 * M_PI,  // min_alpha
      20,           // max_thrust
      0.05          // min_thrust
  });
  std::vector<ruddermaindata> v_ruddermaindata;

  controllerRTdata<m, n> _controllerRTdata{
      Eigen::Matrix<double, n, 1>::Zero(),  // tau
      Eigen::Matrix<double, n, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, m, 1>::Zero(),  // u
      Eigen::Matrix<int, m, 1>::Zero(),     // rotation
      Eigen::Matrix<double, m, 1>::Zero(),  // alpha
      Eigen::Matrix<int, m, 1>::Zero()      // alpha_deg
  };

  motorRTdata<m> testmotorRTdata;

  motorclient _motorclient;
  _motorclient.startup_socket_client(testmotorRTdata);
  thrustallocation<m, index_actuation, n> _thrustallocation(
      _thrustallocationdata, v_tunnelthrusterdata, v_azimuththrusterdata,
      v_ruddermaindata);
  _thrustallocation.initializapropeller(_controllerRTdata);

  sleep(10);

  // desired forces
  const int totalstep = 200;
  Eigen::MatrixXd save_tau = Eigen::MatrixXd::Zero(n, totalstep);
  double angle = 0;
  for (int i = 0; i != 120; ++i) {
    angle = (i + 1) * M_PI / 60;
    save_tau(2, i + 1) = 0.0 * sin(angle) + 0.0 * std::rand() / RAND_MAX;
  }
  save_tau.block(1, 0, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 1) +
                                 0.0 * Eigen::MatrixXd::Random(1, 100);
  save_tau.block(1, 100, 1, 100) = Eigen::MatrixXd::Constant(1, 100, 1) +
                                   0.0 * Eigen::MatrixXd::Random(1, 100);
  save_tau.row(0) = Eigen::MatrixXd::Constant(1, 200, 0);

  for (int i = 0; i != totalstep; ++i) {
    // update tau
    _controllerRTdata.tau = save_tau.col(i);
    // thruster allocation
    _thrustallocation.onestepthrustallocation(_controllerRTdata);

    _motorclient.commandfromcontroller(
        testmotorRTdata.command_alpha, testmotorRTdata.command_rotation,
        _controllerRTdata.alpha_deg, _controllerRTdata.rotation);

    _motorclient.PLCcommunication(testmotorRTdata);
    printf("Positon :");
    for (int j = 0; j < 6; j++) {
      printf("%d  ", testmotorRTdata.feedback_alpha[j]);
    }
    printf("\n");

    printf("Velocity:");
    for (int j = 0; j < 6; j++) {
      printf("%d  ", testmotorRTdata.feedback_rotation[j]);
    }
    printf("\n");

    printf("Torque:");
    for (int j = 0; j < 12; j++) {
      printf("%d  ", testmotorRTdata.feedback_torque[j]);
    }
    printf("\n");
    usleep(1000000);
  }
}

int main() { test(); }