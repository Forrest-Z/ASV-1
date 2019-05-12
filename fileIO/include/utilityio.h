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
#include <fstream>
#include <iostream>
#include <vector>

class utilityio {
 public:
  utilityio() {}
  ~utilityio() {}

 public:
  template <typename T, int m, int n>
  Eigen::Matrix<T, m, n> convertstdvector2EigenMat(const std::vector<T> &_vec) {
    Eigen::Matrix<T, m, n> mat = Eigen::Matrix<T, m, n>::Zero();
    for (int i = 0; i != n; ++i)
      for (int j = 0; j != m; ++j) mat(j, i) = _vec[i * m + j];
    return mat;
  }
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

  // write Eigen matrix into csv file (double)
  void write2csvfile(const std::string &name, const Eigen::MatrixXd &matrix) {
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                           Eigen::DontAlignCols, ", ", "\n");
    std::ofstream file(name.c_str());
    if (file.is_open()) {
      // header in csv
      int num_cols = matrix.cols();
      for (int i = 0; i != (num_cols - 1); ++i)
        file << "column " << i + 1 << ", ";
      file << "column " << num_cols << std::endl;
      // data
      file << matrix.format(CSVFormat);
    }
  }
  // write Eigen matrix into csv file (int)
  void write2csvfile(const std::string &name, const Eigen::MatrixXi &matrix) {
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                           Eigen::DontAlignCols, ", ", "\n");
    std::ofstream file(name.c_str());
    if (file.is_open()) {
      // header in csv
      int num_cols = matrix.cols();
      for (int i = 0; i != (num_cols - 1); ++i)
        file << "column " << i + 1 << ", ";
      file << "column " << num_cols << std::endl;
      // data
      file << matrix.format(CSVFormat);
    }
  }
};

#endif /*UTILITYIO_H*/