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

class lineofsight {
 public:
  explicit lineofsight(double _los_radius, double _capture_radius = 0)
      : los_radius(_los_radius),
        capture_radius(_capture_radius),
        trackerror(Eigen::Vector2d::Zero()),
        R(Eigen::Matrix2d::Zero()) {}
  lineofsight() = delete;
  ~lineofsight() {}

  bool judgewaypoint(const Eigen::Vector2d &_vesselposition,
                     const Eigen::Vector2d &_wp1) {
    Eigen::Vector2d _error = _vesselposition - _wp1;
    double _distance =
        std::sqrt(std::pow(_error(0), 2) + std::pow(_error(1), 2));
    if (_distance > capture_radius) return true;
    return false;
  }

  // compute the orientation of LOS vector and cross-track error
  double computelospoint(const Eigen::Vector2d &_vesselposition,
                         const Eigen::Vector2d &_wp0,
                         const Eigen::Vector2d &_wp1) {
    //
    auto delta_pos = _wp1 - _wp0;

    double thetaK = std::atan(delta_pos(1) / delta_pos(0));
    if (delta_pos(0) < 0) thetaK += M_PI;

    // rotation matrix
    computeR(thetaK);

    // track error
    trackerror = R.transpose() * (_vesselposition - _wp0);

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

  Eigen::Vector2d gettrackerror() const { return trackerror; }

  void setcaptureradius(double _captureradius) {
    capture_radius = _captureradius;
  }
  void setlosradius(double _radius) { los_radius = _radius; }

 private:
  double los_radius;
  double capture_radius;
  Eigen::Vector2d trackerror;
  Eigen::Matrix2d R;

  // compute the rotation matrix
  void computeR(double theta) {
    double svalue = std::sin(theta);
    double cvalue = std::cos(theta);
    R(0, 0) = cvalue;
    R(0, 1) = -svalue;
    R(1, 0) = svalue;
    R(1, 1) = cvalue;
  }  // computeR
};

#endif /* _LINEOFSIGHT_H_ */