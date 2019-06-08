/*
*****************************************************************************
* testguicommunication.cc:
* unit test for gui communication
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*****************************************************************************
*/

#include "guiserver.h"

INITIALIZE_EASYLOGGINGPP

int main() {
  constexpr int num_thruster = 6;
  constexpr int dim_controlspace = 3;

  guiserver<num_thruster, dim_controlspace> _guiserver;

  plannerRTdata _plannerRTdata{
      Eigen::Vector3d::Zero(),  // setpoint
      Eigen::Vector3d::Zero(),  // v_setpoint
      Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),  // waypoint1
      Eigen::Vector3d::Zero()   // command
  };

  // realtime parameters of the estimators
  estimatorRTdata _estimatorRTdata{
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      (Eigen::Matrix<double, 6, 1>() << 351042.103705, 3433888.98025, 0.626135,
       0.1, 0, -0.003678)
          .finished(),
      Eigen::Vector3d::Zero(),             // p_error
      Eigen::Vector3d::Zero(),             // v_error
      Eigen::Vector3d::Zero(),             // BalphaU
      Eigen::Matrix<double, 6, 1>::Zero()  // motiondata_6dof
  };

  gpsRTdata gps_data{
      0,                // date
      0,                // time
      0,                // heading
      0,                // pitch
      0,                // roll
      31.028788,        // latitude
      121.439365,       // longitude
      0,                // altitude
      0,                // Ve
      0,                // Vn
      0,                // Vu
      0,                // base_line
      0,                // NSV1
      0,                // NSV2
      'a',              // status
      {'a', 'b', '0'},  // check
      0,                // UTM_x
      0                 // UTM_y
  };
  indicators _indicators{
      0,                // gui_connection
      0,                // joystick_connection
      DYNAMICPOSITION,  // controlmode
      WINDON,           // windstatus
  };

  motorRTdata<6> testmotorRTdata = {};
  timecounter _timer;
  while (1) {
    static int count = 0;
    for (int i = 0; i != num_thruster; ++i) {
      testmotorRTdata.feedback_alpha[i] = 30 + i;
    }
    for (int i = 0; i != num_thruster; ++i) {
      testmotorRTdata.feedback_rotation[i] = 300 + i + count;
    }
    for (int i = 0; i != (2 * num_thruster); ++i) {
      testmotorRTdata.feedback_torque[i] = 3000;
    }
    testmotorRTdata.feedback_allinfo = 0;
    count++;
    _guiserver.guicommunication(_indicators, _estimatorRTdata, _plannerRTdata,
                                gps_data, testmotorRTdata);
    std::cout << _guiserver;
  }
}