/*
***********************************************************************
* jsonparse.h:
* Parse JSON file for USV
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef JSONPARSE_H
#define JSONPARSE_H
#include <fstream>
#include <iomanip>
#include <iostream>
#include <json.hpp>
#include <string>
#include <vector>
#include "utilityio.h"

/*
global coordinate (GLOBAL), which is an inertial reference frame;
body-fixed coordinate (BODY); whose origin located at the stern
body-fixed coordinate (BODY-G), whose origin located at the center of gravity
*/

class jsonparse {
 public:
  jsonparse() {}
  ~jsonparse() {}

 private:
  bool index_actuation;        // 1: fully-actuated; 0: underactuated
  int n;                       // 3: fully-actuated; 2: underactuated
  std::string dbsavedir;       // directory for database file
  std::string logsavedir;      // directory for log file
  int control_rate;            // Hz
  double control_sample_time;  // s
  Eigen::MatrixXd Mass;        // mass matrix, in the BODY-G
  Eigen::MatrixXd AddedMass;
  Eigen::MatrixXd TotalMass;
  Eigen::MatrixXd Damping;  // damping matrix, in the BODY-G
  Eigen::VectorXd CoG;      // position of CoG, in the BODY

  Eigen::VectorXd thrustlimit_p;  // positive
  Eigen::VectorXd thrustlimit_n;  // negative
  int num_thruster;               // # of thrusters
  int num_constraints;            // # of constraints in control allocation
  int nun_var;                    // # of variables in control allocation
  // the number of constraints
  Eigen::VectorXd x_thruster;  // position of each thruster in the BODY-G
  Eigen::VectorXd y_thruster;
};

#endif /* JSONPARSE_H */
