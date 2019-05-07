/*
***********************************************************************
* threadloop.h: thread-based DP controller and network
* function to run the whole loop on server (including TCP/IP server,
* senser, estimator, controller, planner, database, etc).
* This header file can be read by C++ compilers
*
* 碧凌: 上古神兽名。碧凌的徒弟便是上古四大神兽：青龙、白虎、朱雀、玄武。
* 相传碧凌是身穿紧身碧衣，满头青丝且瞳目为碧绿之色的太极神兽，由此而生。
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _THREADLOOP_H_
#define _THREADLOOP_H_

#include <chrono>
#include <thread>
#include "estimator.h"
#include "gps.h"
#include "timecounter.h"

class threadloop {
 public:
  threadloop()
      : _estimator(_vessel, _estimatordata), _gpsimu(51, true, 115200) {}
  ~threadloop() {}

  void testthread() {
    std::thread gps_thread(&threadloop::gpsimuloop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    gps_thread.join();
    estimator_thread.join();
  }

 private:
  // mass property of vessel (TODO: read from json)
  vessel _vessel{
      (Eigen::Matrix3d() << 500, 0, 0, 0, 500, 344, 0, 344, 1000)
          .finished(),  // Mass
      (Eigen::Matrix3d() << 123, 0, 0, 0, 206, 100, 0, 100, 298)
          .finished(),  // added Mass
      (Eigen::Matrix3d() << 17, 0, 0, 0, 20, 0, 0, 0, 100)
          .finished(),                              // damping
      (Eigen::Vector2d() << 1.9, 0).finished(),     // cog
      (Eigen::Vector2d() << -5.0, 6.0).finished(),  // x_thrust
      (Eigen::Vector2d() << -2.0, 3.0).finished(),  // y_thrust
      (Eigen::Vector2d() << -2.0, 3.0).finished(),  // mz_thrust
      (Eigen::Vector2d() << -3.0, 5.0).finished(),  // surge_v
      (Eigen::Vector2d() << -1.0, 1.0).finished(),  // sway_v
      (Eigen::Vector2d() << -1.0, 1.0).finished(),  // yaw_v
      (Eigen::Vector2d() << -0.5, 0.5).finished()   // roll_v
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
  // constant parameters in state estimation
  estimatordata _estimatordata{
      0.1,  // sample_time
      true  // kalman_use
  };
  // real time GPS/IMU data
  gpsRTdata gps_data{
      0,  // date
      0,  // time
      0,  // heading
      0,  // pitch
      0,  // roll
      0,  // latitude
      0,  // longitude
      0,  // altitude
      0,  // Ve
      0,  // Vn
      0,  // Vu
      0,  // base_line
      0,  // NSV1
      0,  // NSV2
      0,  // status
      0,  // check
      0,  // UTM_x
      0   // UTM_y
  };

  estimator _estimator;
  gpsimu _gpsimu;

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
      std::cerr << "Unhandled Exception: " << e.what() << std::endl;
    }
  }

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

    double desiredheading = 0;
    Eigen::Vector3d setpoints = Eigen::Vector3d::Zero();
    Eigen::Vector3d vsetpoints = Eigen::Vector3d::Zero();
    while (1) {
      _estimator.estimatestate(_estimatorRTdata, gps_data.UTM_x, gps_data.UTM_y,
                               gps_data.altitude, gps_data.roll, gps_data.pitch,
                               gps_data.heading, gps_data.Ve, gps_data.Vn,
                               desiredheading);
      _estimator.estimateerror(_estimatorRTdata, setpoints, vsetpoints);
      std::cout << _estimatorRTdata.Measurement << std::endl;
      std::cout << _estimatorRTdata.CTB2G << std::endl;
      // std::cout << _estimatorRTdata.Measurement << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }
  }

};

#endif /* _THREADLOOP_H_ */