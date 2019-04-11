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

  void utest() {
    Eigen::MatrixXd mat(3, 4);
    mat << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
    Eigen::VectorXd Vec(12);
    Vec << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
    std::cout << mat << std::endl;
    std::vector<double> mat2vec = convertEigenMat2stdvector(mat);
    std::vector<double> vec2vec = convertEigenVec2stdvector(Vec);
    for (std::vector<double>::iterator it = mat2vec.begin();
         it != mat2vec.end(); ++it) {
      std::cout << *it << '\n';
    }
    for (std::vector<double>::iterator it = vec2vec.begin();
         it != vec2vec.end(); ++it) {
      std::cout << *it << '\n';
    }
    Eigen::MatrixXd vec2mat = convertstdvector2EigenMat(mat2vec, 3, 4);
    std::cout << vec2mat << std::endl;
  }

 private:
  Eigen::MatrixXd convertstdvector2EigenMat(const std::vector<double> &_vec,
                                            int nrow, int ncol) {
    Eigen::MatrixXd mat(nrow, ncol);
    for (int i = 0; i != ncol; ++i)
      for (int j = 0; j != nrow; ++j) mat(j, i) = _vec[i * nrow + j];
    return mat;
  }
  std::vector<double> convertEigenMat2stdvector(const Eigen::MatrixXd &_mat) {
    std::vector<double> vec(_mat.data(),
                            _mat.data() + _mat.rows() * _mat.cols());
    return vec;
  }
  std::vector<double> convertEigenVec2stdvector(const Eigen::VectorXd &_vec) {
    std::vector<double> vec(_vec.data(), _vec.data() + _vec.size());
    return vec;
  }
};

#endif /*UTILITYIO_H*/