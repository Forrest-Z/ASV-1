/*
***********************************************************************
* utilityio.h:
* The utility library for file io
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef UTILITYIO_H
#define UTILITYIO_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

class utilityio {
 public:
  utilityio() {}
  ~utilityio() {}

 public:
  // convert std vector to Eigen3 Matrix (by column)
  Eigen::MatrixXd convertstdvector2EigenMat(const std::vector<double> &_vec,
                                            int nrow, int ncol) {
    Eigen::MatrixXd mat(nrow, ncol);
    for (int i = 0; i != ncol; ++i)
      for (int j = 0; j != nrow; ++j) mat(j, i) = _vec[i * nrow + j];
    return mat;
  }
  // convert Eigen3 Matrix to std vector (by column)
  std::vector<double> convertEigenMat2stdvector(const Eigen::MatrixXd &_mat) {
    std::vector<double> vec(_mat.data(),
                            _mat.data() + _mat.rows() * _mat.cols());
    return vec;
  }
  // convert Eigen3 vector to std vector
  std::vector<double> convertEigenVec2stdvector(const Eigen::VectorXd &_vec) {
    std::vector<double> vec(_vec.data(), _vec.data() + _vec.size());
    return vec;
  }
};

#endif /*UTILITYIO_H*/