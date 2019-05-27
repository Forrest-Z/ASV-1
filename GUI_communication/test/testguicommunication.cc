/*
*****************************************************************************
* testguicommunication.cc:
* unit test for timer in milliseconds
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*****************************************************************************
*/

#include "guiserver.h"

INITIALIZE_EASYLOGGINGPP

int main() {
  constexpr int num_thruster = 1;
  constexpr int dim_controlspace = 3;

  guiserver<num_thruster, dim_controlspace> _guiserver;

  plannerRTdata _plannerRTdata{
      Eigen::Vector3d::Zero(),  // setpoint
      Eigen::Vector3d::Zero(),  // v_setpoint
      Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),  // waypoint1
      Eigen::Vector3d::Zero()   // command
  };

  controllerRTdata<num_thruster, dim_controlspace> _controllerRTdata{
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // tau
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // u
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // rotation
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // alpha
      Eigen::Matrix<int, num_thruster, 1>::Zero()          // alpha_deg
  };

  // realtime parameters of the estimators
  estimatorRTdata _estimatorRTdata{
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero(),              // BalphaU
      Eigen::Matrix<double, 6, 1>::Zero()   // motiondata_6dof
  };

  timecounter _timer;
  while (1) {
    _guiserver.convertalldata2string(_controllerRTdata, _estimatorRTdata,
                                     _plannerRTdata);
    std::cout << _timer.timeelapsed() << std::endl;
    std::cout << _guiserver;
  }
}