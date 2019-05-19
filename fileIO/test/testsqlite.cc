/*
*******************************************************************************
* testsqlite.cc: unit test for modern sqlite wrapper
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "database.h"

INITIALIZE_EASYLOGGINGPP

int main() {
  const int m = 3;
  const int n = 3;
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  LOG(INFO) << "The program has started!";

  database<m, n> _sqlitetest("dbtest.db");
  _sqlitetest.initializetables();
  // real time GPS/IMU data
  gpsRTdata gps_data{
      0,                // date
      0,                // time
      0,                // heading
      0,                // pitch
      0,                // roll
      0,                // latitude
      0,                // longitude
      0,                // altitude
      0,                // Ve
      0,                // Vn
      0,                // Vu
      0,                // base_line
      0,                // NSV1
      0,                // NSV2
      '0',              // status
      {'a', 'b', '0'},  // check
      0,                // UTM_x
      0                 // UTM_y
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
  controllerRTdata<m, n> _controllerRTdata{
      (Eigen::Matrix<double, n, 1>() << 0, 0, 1).finished(),    // tau
      Eigen::Matrix<double, n, 1>::Zero(),                      // BalphaU
      (Eigen::Matrix<double, m, 1>() << 0, 0.5, 0).finished(),  // u
      // vectormi()::Zero(),                    // rotation
      (Eigen::Matrix<int, m, 1>() << 100, 500, 400).finished(),
      (Eigen::Matrix<double, m, 1>() << M_PI / 2, -M_PI / 2, M_PI * 2 / 3)
          .finished(),                  // alpha
      Eigen::Matrix<int, m, 1>::Zero()  // alpha_deg

  };
  for (int i = 0; i != 3; ++i) {
    _sqlitetest.update_gps_table(gps_data);
    _sqlitetest.update_estimator_table(_estimatorRTdata);
    _sqlitetest.update_controller_table(_controllerRTdata);
  }

  LOG(INFO) << "Shutting down.";
  return 0;
}