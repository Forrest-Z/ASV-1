/*
*******************************************************************************
* vesseldata.h:
* define the property of each vessel
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _VESSELDATA_H_
#define _VESSELDATA_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

// 船体的参数
struct vessel {
  // mass property
  Eigen::Matrix3d Mass;
  Eigen::Matrix3d AddedMass;
  Eigen::Matrix3d Damping;
  Eigen::Vector2d cog;

  // thrust limit
  Eigen::Vector2d x_thrust;   // min, max
  Eigen::Vector2d y_thrust;   // min, max
  Eigen::Vector2d mz_thrust;  // min, max

  // velocity limit
  Eigen::Vector2d surge_v;  // min, max
  Eigen::Vector2d sway_v;   // min, max
  Eigen::Vector2d yaw_v;    // min, max
  Eigen::Vector2d roll_v;   // min, max

  // geometry
  double L;  // total length of vessel
  double B;  // total width of vessel
};

#endif /* _VESSELDATA_H_ */
