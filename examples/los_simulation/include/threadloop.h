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
#include "planner.h"
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
        _planner(_jsonparse.getplannerdata()),
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
    std::thread planner_thread(&threadloop::plannerloop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);

    planner_thread.detach();
    controller_thread.detach();
    estimator_thread.detach();
    sql_thread.detach();
  }

 private:
  // json
  jsonparse<num_thruster, dim_controlspace> _jsonparse;
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

  planner _planner;
  estimator<indicator_kalman> _estimator;

  controller<10, num_thruster, indicator_actuation, dim_controlspace>
      _controller;
  windcompensation _windcompensation;
  database<num_thruster, dim_controlspace> _sqlite;

  void intializethreadloop() {
    _controller.initializecontroller(_controllerRTdata);
    _sqlite.initializetables();
  }

  void plannerloop() {
    Eigen::MatrixXd wpts(2, 8);
    wpts.col(0) << 0.372, -0.181;
    wpts.col(1) << -0.628, 1.320;
    wpts.col(2) << 0.372, 2.820;   //
    wpts.col(3) << 1.872, 3.320;   //
    wpts.col(4) << 6.872, -0.681;  //
    wpts.col(5) << 8.372, -0.181;  //
    wpts.col(6) << 9.372, 1.320;   //
    wpts.col(7) << 8.372, 2.820;

    _plannerRTdata.waypoint0 = wpts.col(0);
    _plannerRTdata.waypoint1 = wpts.col(1);
    _planner.setconstantspeed(_plannerRTdata, 0.1);

    int index_wpt = 1;
    while (1) {
      // if (_planner.switchwaypoint(_plannerRTdata,
      //                             _estimatorRTdata.State.head(2),
      //                             wpts.col(index_wpt))) {
      //   ++index_wpt;
      //   std::cout << index_wpt << std::endl;
      // }
      if (index_wpt == 8) break;
      _planner.pathfollowLOS(_plannerRTdata, _estimatorRTdata.State.head(2));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  // plannerloop

  void controllerloop() {
    _controller.setcontrolmode(AUTOMATIC);
    timecounter timer_controler;
    long int elapsed_time = 0;
    while (1) {
      _controller.controlleronestep(
          _controllerRTdata, _windcompensation.getwindload(),
          _estimatorRTdata.p_error, _estimatorRTdata.v_error,
          _plannerRTdata.command);

      elapsed_time = timer_controler.timeelapsed();
      // std::cout << elapsed_time << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // controllerloop

  // loop to give real time state estimation
  void estimatorloop() {
    _estimator.setvalue(_estimatorRTdata, -0.2, 0, 0, 0, 0, 102, 0, 0);
    CLOG(INFO, "GPS") << "initialation successful!";

    while (1) {
      _estimator.updateestimatedforce(
          _estimatorRTdata, _controllerRTdata.BalphaU,
          _windcompensation.computewindload(0, 0).getwindload());
      _estimator.estimatestate(_estimatorRTdata, _plannerRTdata.setpoint(2));
      _estimator.estimateerror(_estimatorRTdata, _plannerRTdata.setpoint,
                               _plannerRTdata.v_setpoint);
      // std::cout << _estimatorRTdata.Measurement << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // estimatorloop()

  // loop to save real time data using sqlite3
  void sqlloop() {
    while (1) {
      _sqlite.update_planner_table(_plannerRTdata);
      _sqlite.update_estimator_table(_estimatorRTdata);
      _sqlite.update_controller_table(_controllerRTdata);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // sqlloop()
};

#endif /* _THREADLOOP_H_ */