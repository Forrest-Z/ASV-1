/*
*******************************************************************************
* lineofsight.h:
* path following using line of sight algorithm
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _LINEOFSIGHT_H_
#define _LINEOFSIGHT_H_
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "plannerdata.h"

class lineofsight {
 public:
  explicit lineofsight(double _los_radius, double _capture_radius = 0)
      : los_radius(_los_radius),
        capture_radius(_capture_radius),
        pos_p0(Eigen::Vector2d::Zero()),
        pos_p1(Eigen::Vector2d::Zero()),
        trackerror(Eigen::Vector2d::Zero()) {}
  lineofsight() = delete;
  ~lineofsight() {}

  void testlos(double _vesselx, double _vessely) {
    Eigen::Vector2d vp = Eigen::Vector2d::Zero();
    vp << _vesselx, _vessely;
    double test = computelospoint(vp);
    std::cout << test << std::endl;
  }

  Eigen::Vector2d gettrackerror() const { return trackerror; }
  void setcaptureradius(double _captureradius) {
    capture_radius = _captureradius;
  }
  void setlosradius(double _radius) { los_radius = _radius; }
  void setpos_p0(const Eigen::Vector2d &_wp) { pos_p0 = _wp; }
  void setpos_p1(const Eigen::Vector2d &_wp) { pos_p1 = _wp; }

 private:
  double los_radius;
  double capture_radius;
  Eigen::Vector2d pos_p0;
  Eigen::Vector2d pos_p1;
  Eigen::Vector2d trackerror;
  // compute the orientation of LOS vector and cross-track error
  double computelospoint(const Eigen::Vector2d &_vesselposition) {
    //
    auto delta_pos = pos_p1 - pos_p0;

    double thetaK = std::atan(delta_pos(1) / delta_pos(0));
    if (delta_pos(0) < 0) thetaK += M_PI;

    // rotation matrix
    Eigen::Matrix2d R = computeR(thetaK);

    // track error
    trackerror = R.transpose() * (_vesselposition - pos_p0);

    double e = trackerror(1);  // cross-track error
    double thetar = 0;
    if (e > los_radius)
      thetar = -M_PI / 2;
    else if (e < -los_radius)
      thetar = M_PI / 2;
    else
      thetar = std::asin(-e / los_radius);
    return thetar + thetaK;
  }  // computelospoint
  // compute the rotation matrix
  Eigen::Matrix2d computeR(double theta) {
    double svalue = std::sin(theta);
    double cvalue = std::cos(theta);
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = cvalue;
    R(0, 1) = -svalue;
    R(1, 0) = svalue;
    R(1, 1) = cvalue;
    return R;
  }  // computeR
};

#endif /* _LINEOFSIGHT_H_ */