/*
***********************************************************************
* lowpass.h: data processing including outlier removal,
* low pass filtering, etc
* This header file can be read by C++ compilers
*
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _LOWPASS_H_
#define _LOWPASS_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include "vesseldata.h"

template <int num_lowpass>
class lowpass {
  using vectorlp = Eigen::Matrix<double, num_lowpass, 1>;

 public:
  explicit lowpass(const vessel &_vessel) : averagevector(vectorlp::Zero()) {
    initializekalman(_vessel);
  }
  lowpass() = delete;
  ~lowpass() {}

  double movingaverage(double _newstep) {
    // pop_front
    vectorlp t_average = vectorlp::Zero();
    t_average.head(num_lowpass - 1) = averagevector.tail(num_lowpass - 1);
    // push back
    t_average(num_lowpass - 1) = _newstep;
    averagevector = t_average;
    // calculate the mean value
    return averagevector.mean();
  }

 private:
  vectorlp averagevector;
};

#endif /* _DATAPROCESS_H_ */