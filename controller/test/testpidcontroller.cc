/*
*******************************************************************************
* testpidcontroller.cc:
* unit test for thrust allocation
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "pidcontroller.h"

int main() {
  const int L = 30;
  const int n = 3;

  using vectornd = Eigen::Matrix<double, n, 1>;
  std::vector<pidcontrollerdata> v_pidcontrollerdata;
  v_pidcontrollerdata.reserve(n);
  v_pidcontrollerdata.push_back({1, 1, 1, 0.1, -1, 3});
  v_pidcontrollerdata.push_back({1, 2, 3, 0.1, -1, 3});
  v_pidcontrollerdata.push_back({1, 2, 10, 0.1, -2, 4});

  pidcontroller<L, n> _pidcontroller(v_pidcontrollerdata, 0.01);
  vectornd error;
  vectornd derror;
  vectornd feedforward;
  error << 1, 3, 4;
  derror << 0.9, 1, 4;
  feedforward << 0.4, -1, 0;
  for (int i = 0; i != 100; ++i) {
    vectornd _ddd =
        _pidcontroller.calculategeneralizeforce(error, derror, feedforward);

    std::cout << "step " << i << std::endl;
    std::cout << _ddd << std::endl;
  }
}