/*
***********************************************************************
* threadloop.h: thread-based DP controller and network
* function to run the whole loop on server (including TCP/IP server,
* senser, estimator, controller, planner, database, etc).
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _THREADLOOP_H_
#define _THREADLOOP_H_

#include <chrono>
#include <thread>
#include "controller.h"
#include "database.h"
#include "easylogging++.h"
#include "estimator.h"
#include "windcompensation.h"
// #include "gps.h"(
#include "jsonparse.h"
#include "timecounter.h"

constexpr int num_thruster = 1;
constexpr int dim_controlspace = 3;
constexpr USEKALMAN indicator_kalman = KALMANON;
constexpr ACTUATION indicator_actuation = UNDERACTUATED;

class threadloop {
 public:
  threadloop()
      : _jsonparse(
            "/home/scar1et/Coding/ASV/examples/los_simulation/properties/"
            "property.json"),
        _estimator(_jsonparse.getvessel(), _jsonparse.getestimatordata()),
        _controller(_jsonparse.getcontrollerdata(), _jsonparse.getpiddata(),
                    _jsonparse.getthrustallocationdata(),
                    _jsonparse.gettunneldata(), _jsonparse.getazimuthdata(),
                    _jsonparse.getmainrudderdata()),
        _sqlite(_jsonparse.getsqlitedata()) {
    intializethreadloop();
  }
  ~threadloop() {}

  void testthread() {
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);

    controller_thread.detach();
    estimator_thread.detach();
    sql_thread.detach();
  }

 private:
  // json
  jsonparse<num_thruster, dim_controlspace> _jsonparse;

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

  estimator<indicator_kalman> _estimator;

  controller<10, num_thruster, indicator_actuation, dim_controlspace>
      _controller;
  windcompensation _windcompensation;
  database<num_thruster, dim_controlspace> _sqlite;

  void intializethreadloop() {
    _controller.initializecontroller(_controllerRTdata);
    _sqlite.initializetables();
  }

  void controllerloop() {
    Eigen::Matrix<double, dim_controlspace, 1> command =
        Eigen::Matrix<double, dim_controlspace,
                      1>::Zero();  // TODO: command from planner
    _controller.setcontrolmode(MANUAL);

    int counter = 0;
    double angle = 0;
    while (1) {
      ++counter;
      angle = (counter + 1) * M_PI / 60;
      command(0) = 0.2;
      command(1) = 0.2;
      command(2) = 0.5 * sin(angle) + 0.1 * std::rand() / RAND_MAX;
      _controller.controlleronestep(
          _controllerRTdata, _windcompensation.getwindload(),
          _estimatorRTdata.p_error, _estimatorRTdata.v_error, command);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // controllerloop

  // loop to give real time state estimation
  void estimatorloop() {
    _estimator.setvalue(_estimatorRTdata, -0.2, 0, 0, 0, 0, 1.78, 0.1, 0);
    CLOG(INFO, "GPS") << "initialation successful!";

    double desiredheading = 0;                             // TODO: planner
    Eigen::Vector3d setpoints = Eigen::Vector3d::Zero();   // TODO: planner
    Eigen::Vector3d vsetpoints = Eigen::Vector3d::Zero();  // TODO: planner
    while (1) {
      _estimator.updateestimatedforce(
          _estimatorRTdata, _controllerRTdata.BalphaU,
          _windcompensation.computewindload(0, 0).getwindload());
      _estimator.estimatestate(_estimatorRTdata, 0, 0, 0, 0, 0, 0, 0, 0,
                               desiredheading);
      _estimator.estimateerror(_estimatorRTdata, setpoints, vsetpoints);
      // std::cout << _estimatorRTdata.Measurement << std::endl;
      // std::cout << _estimatorRTdata.CTB2G << std::endl;
      // std::cout << _estimatorRTdata.Measurement << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // estimatorloop()

  // loop to save real time data using sqlite3
  void sqlloop() {
    while (1) {
      _sqlite.update_estimator_table(_estimatorRTdata);
      _sqlite.update_controller_table(_controllerRTdata);
    }
  }  // sqlloop()
};

#endif /* _THREADLOOP_H_ */