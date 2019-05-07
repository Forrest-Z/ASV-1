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

using std::setprecision;

class threadloop {
 public:
  threadloop()
      : _estimator(_vessel, _estimatordata), _gpsimu(51, true, 115200) {}
  ~threadloop() {}

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
  void gpsimuloop() {
    std::string buffer;
    try {
      while (1) {
        gps_data = _gpsimu.gpsonestep().getgpsRTdata();
        buffer = _gpsimu.getserialbuffer();
        std::cout << "buffer=    " << buffer;
        std::cout << "date:      " << gps_data.date << std::endl;
        std::cout << "time:      " << std::fixed << setprecision(3)
                  << gps_data.time << std::endl;
        std::cout << "heading:   " << std::fixed << setprecision(3)
                  << gps_data.heading << std::endl;
        std::cout << "pitch:     " << std::fixed << setprecision(3)
                  << gps_data.pitch << std::endl;
        std::cout << "roll:      " << std::fixed << setprecision(3)
                  << gps_data.roll << std::endl;
        std::cout << "latitud:   " << std::fixed << setprecision(7)
                  << gps_data.latitude << std::endl;
        std::cout << "longitude: " << std::fixed << setprecision(7)
                  << gps_data.longitude << std::endl;
        std::cout << "UTM_x:     " << std::fixed << setprecision(7)
                  << gps_data.UTM_x << std::endl;
        std::cout << "UTM_y:     " << std::fixed << setprecision(7)
                  << gps_data.UTM_y << std::endl;
        std::cout << "altitude:  " << std::fixed << setprecision(2)
                  << gps_data.altitude << std::endl;
        std::cout << "speed_v:   " << std::fixed << setprecision(3)
                  << gps_data.Ve << std::endl;
        std::cout << "speed_u:   " << std::fixed << setprecision(3)
                  << gps_data.Vn << std::endl;
        std::cout << "speed_n:   " << std::fixed << setprecision(3)
                  << gps_data.Vu << std::endl;
        std::cout << "basinLine  " << std::fixed << setprecision(3)
                  << gps_data.base_line << std::endl;
        std::cout << "NSV1:      " << gps_data.NSV1 << std::endl;
        std::cout << "NSV2:      " << gps_data.NSV2 << std::endl;
        printf("status: %c\n", gps_data.status);
        printf("check: %s\n", gps_data.check);

        switch (gps_data.status) {
          case '0':
            std::cout << "Satus:     GPS初始化" << std::endl;
            break;
          case '1':
            std::cout << "Satus:     粗对准" << std::endl;
            break;
          case '2':
            std::cout << "Satus:     精对准" << std::endl;
            break;
          case '3':
            std::cout << "Satus:     GPS定位" << std::endl;
            break;
          case '4':
            std::cout << "Satus:     GPS定向" << std::endl;
            break;
          case '5':
            std::cout << "Satus:     GPS RTK" << std::endl;
            break;
          case 'B':
            std::cout << "Satus:     差分定向" << std::endl;
            break;
          default:
            std::cout << "Satus:     状态未知" << std::endl;
        }
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

    } catch (std::exception& e) {
      std::cerr << "Unhandled Exception: " << e.what() << std::endl;
    }
  }

  void estimatorloop() {
    while (1) {
      if (gps_data.status = 'B') {
        _estimator.setvalue(_estimatorRTdata, gps_data.UTM_x, gps_data.UTM_y,
                            gps_data.altitude, gps_data.roll, gps_data.pitch,
                            gps_data.heading, gps_data.Ve, gps_data.Vn);
        break;
      }
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
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
  }

};

#endif /* _THREADLOOP_H_ */