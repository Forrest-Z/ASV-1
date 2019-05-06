#include "estimator.h"

int main() {
  estimatorRTdata _estimatorRTdata{
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // state
      Eigen::Matrix<double, 3, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, 6, 1>::Zero()   // motiondata_6dof
  };
}