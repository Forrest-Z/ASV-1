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
  const Eigen::Matrix3d Mass;
  const Eigen::Matrix3d AddedMass;
  const Eigen::Matrix3d Damping;

  // thrust limit
  const Eigen::Vector2d x_thrust;   // min, max
  const Eigen::Vector2d y_thrust;   // min, max
  const Eigen::Vector2d mz_thrust;  // min, max

  // velocity limit
  const Eigen::Vector2d surge_v;  // min, max
  const Eigen::Vector2d sway_v;   // min, max
  const Eigen::Vector2d yaw_v;    // min, max
};

#endif /* _VESSELDATA_H_ */
