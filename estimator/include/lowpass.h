/*
***********************************************************************
* lowpass.h: low pass filtering
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

template <int num_lowpass>
class lowpass {
  using vectorlp = Eigen::Matrix<double, num_lowpass, 1>;

 public:
  lowpass() : averagevector(vectorlp::Zero()) {}
  ~lowpass() {}

  // assign the same value to all elements of "averagevector"
  void setaveragevector(double _initialvalue) {
    averagevector = vectorlp::Constant(_initialvalue);
  }
  // low pass filtering using moving average method
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

#endif /* _LOWPASS_H_ */