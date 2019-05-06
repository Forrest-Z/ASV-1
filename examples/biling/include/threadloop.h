/*
***********************************************************************
* threadloop.h: thread-based DP controller and network
* function to run the whole loop on server (including PN server,
* 6D motion capture, Kalman, PID, thruster allocation, joystick,
* save2sqlite, viewer).
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _THREADLOOP_H_
#define _THREADLOOP_H_

#include "gps.h"
#include "kalmanfilter.h"

class threadloop {
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
      Vector6d::Zero(),             // position
      Vector6d::Zero(),             // measurement
      Vector6d::Zero(),             // state
      Vector6d::Zero(),             // state4control
      Eigen::Vector3d::Zero(),      // setPoints
      0,                            // index for step point
      Eigen::Matrix3d::Identity(),  // CTG2B
      Eigen::Matrix3d::Identity(),  // CTB2G
      Eigen::Vector3d::Zero(),      // tau
      Eigen::Vector3d::Zero(),      // feedforward force
      Eigen::Vector3d::Zero(),      // BalphaU
      (Eigen::Vector3d() << -M_PI / 2, M_PI / 10, -M_PI / 4)
          .finished(),                                   // alpha
      Eigen::Vector3i::Zero(),                           // alpha_deg
      (Eigen::Vector3d() << 0.01, 0.2, 0.2).finished(),  // u
      Eigen::Vector3i::Zero()                            // rotation
  };
};
#endif /* _THREADLOOP_H_ */