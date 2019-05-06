/*
***********************************************************************
* outlierremove.h: outlier removal
* This header file can be read by C++ compilers
*
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef OUTLIERREMOVE_H
#define OUTLIERREMOVE_H

#include <Eigen/Core>
#include <Eigen/Dense>

class outlierremove {
 public:
  explicit outlierremove(double vmax, double vmin, double sample_time,
                         double initial_value = 0)
      : delta_max(vmax * sample_time),
        delta_min(vmin * sample_time),
        last_value(initial_value) {
    initializekalman(_vessel);
  }
  outlierremove() = delete;
  ~outlierremove() {}

  double removeoutlier(double _newvalue) {
    double delta = _newvalue - last_value;
    if ((delta_min < delta) && (delta < delta_max)) {
      last_value = _newvalue;
      return _newvalue;
    } else {
      return last_value;
    }
  }

 private:
  const double delta_max;
  const double delta_min;
  double last_value;
};

#endif /* OUTLIERREMOVE_H */