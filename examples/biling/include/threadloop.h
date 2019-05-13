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
#include "gps.h"
#include "jsonparse.h"
#include "timecounter.h"
const int num_thruster = 3;
const int dim_controlspace = 3;

class threadloop {
 public:
  threadloop()
      : _jsonparse(
            "/home/scar1et/Coding/ASV/examples/biling/properties/"
            "property.json"),
        _estimator(_jsonparse.getvessel(), _jsonparse.getestimatordata()),
        _gpsimu(51, true, 115200),
        _controller(_controllerRTdata, _jsonparse.getcontrollerdata(),
                    _jsonparse.getpiddata(),
                    _jsonparse.getthrustallocationdata(),
                    _jsonparse.gettunneldata(), _jsonparse.getazimuthdata()),
        _sqlite(_jsonparse.getsqlitedata()) {
    intializethreadloop();
  }
  ~threadloop() {}

  void testthread() {
    std::thread gps_thread(&threadloop::gpsimuloop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);

    gps_thread.detach();
    controller_thread.detach();
    estimator_thread.detach();
    sql_thread.detach();
  }

 private:
  // json
  jsonparse<num_thruster, dim_controlspace> _jsonparse;

  controllerRTdata<num_thruster, dim_controlspace> _controllerRTdata{
      (Eigen::Matrix<double, dim_controlspace, 1>() << 0, 0, 1)
          .finished(),                                     // tau
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // BalphaU
      (Eigen::Matrix<double, num_thruster, 1>() << 0.01, 0.2, 0.2)
          .finished(),  // u
      (Eigen::Matrix<int, num_thruster, 1>() << 0, 0, 0)
          .finished(),  // rotation
      (Eigen::Matrix<double, num_thruster, 1>() << M_PI / 2, M_PI / 10,
       -M_PI / 4)
          .finished(),                             // alpha
      Eigen::Matrix<int, num_thruster, 1>::Zero()  // alpha_deg

  };

  windestimation<dim_controlspace> _windestimation{
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // load
      Eigen::Matrix<double, 2, 1>::Zero(),                 // wind_body
      Eigen::Matrix<double, 2, 1>::Zero(),                 // wind_global
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
      'a',              // status
      {'a', 'b', '0'},  // check
      0,                // UTM_x
      0                 // UTM_y
  };

  estimator _estimator;
  gpsimu _gpsimu;
  controller<10, num_thruster, dim_controlspace> _controller;

  database<num_thruster, dim_controlspace> _sqlite;

  void intializethreadloop() { _sqlite.initializetables(); }

  // GPS/IMU
  void gpsimuloop() {
    std::string buffer;
    timecounter gpstimer;
    long int mt_elapsed = 0;

    try {
      while (1) {
        gps_data = _gpsimu.gpsonestep().getgpsRTdata();
        // std::cout << _gpsimu;

        mt_elapsed = gpstimer.timeelapsed();
        // std::this_thread::sleep_for(
        //     std::chrono::milliseconds(100));  //串口不能sleep?
      }

    } catch (std::exception& e) {
      CLOG(ERROR, "GPS serial") << e.what();
    }
  }  // gpsimuloop()

  void controllerloop() {
    Eigen::Matrix<double, dim_controlspace, 1> command =
        Eigen::Matrix<double, dim_controlspace,
                      1>::Zero();  // TODO: command from planner
    while (1) {
      _controller.controlleronestep(_controllerRTdata, _windestimation,
                                    _estimatorRTdata.p_error,
                                    _estimatorRTdata.v_error, command);
    }
  }  // controllerloop

  // loop to give real time state estimation
  void estimatorloop() {
    while (1) {
      if (gps_data.status == 'B') {
        _estimator.setvalue(_estimatorRTdata, gps_data.UTM_x, gps_data.UTM_y,
                            gps_data.altitude, gps_data.roll, gps_data.pitch,
                            gps_data.heading, gps_data.Ve, gps_data.Vn);
        std::cout << "initialation successful!" << std::endl;
        break;
      }
      std::cout << "initialation....." << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    double desiredheading = 0;                             // TODO: planner
    Eigen::Vector3d setpoints = Eigen::Vector3d::Zero();   // TODO: planner
    Eigen::Vector3d vsetpoints = Eigen::Vector3d::Zero();  // TODO: planner
    while (1) {
      _estimator.updateestimatedforce(
          _estimatorRTdata, _controllerRTdata.BalphaU, _windestimation.load);
      _estimator.estimatestate(_estimatorRTdata, gps_data.UTM_x, gps_data.UTM_y,
                               gps_data.altitude, gps_data.roll, gps_data.pitch,
                               gps_data.heading, gps_data.Ve, gps_data.Vn,
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
      _sqlite.update_gps_table(gps_data);
      _sqlite.update_estimator_table(_estimatorRTdata);
      _sqlite.update_controller_table(_controllerRTdata);
    }
  }  // sqlloop()
};

#endif /* _THREADLOOP_H_ */