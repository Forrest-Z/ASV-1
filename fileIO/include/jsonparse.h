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
#include "controllerdata.h"
#include "estimatordata.h"
#include "plannerdata.h"
#include "utilityio.h"
#include "vesseldata.h"
/*
global coordinate (GLOBAL), which is an inertial reference frame;
body-fixed coordinate (BODY); whose origin located at the stern
body-fixed coordinate (BODY-G), whose origin located at the center of gravity
*/

template <int m, int n = 3>
class jsonparse {
  template <int _m, int _n>
  friend std::ostream& operator<<(std::ostream&, const jsonparse<_m, _n>&);

 public:
  jsonparse(const std::string& _jsonname) : jsonname(_jsonname) {}
  ~jsonparse() {}

  void tets() {
    parsejson();
    parsecontrollerdata();
  }

 private:
  std::string jsonname;
  // json file = json::parse(in);
  nlohmann::json file;

  bool index_actuation;        // 1: fully-actuated; 0: underactuated
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

  // controllerdata
  controllerdata controllerdata_input{
      0.1,           // sample_time
      AUTOMATIC,     // controlmode
      WINDON,        // windstatus
      FULLYACTUATED  // index_actuation
  };
  thrustallocationdata thrustallocationdata_input{
      0,  // num_tunnel
      0,  // num_azimuth
      0,  // num_mainrudder
      {}  // index_thrusters
  };
  std::vector<tunnelthrusterdata> tunnelthrusterdata_input;
  std::vector<azimuththrusterdata> azimuththrusterdata_input;
  std::vector<pidcontrollerdata> pidcontrollerdata_input;

  void parsejson() {
    // read a JSON file
    std::ifstream in(jsonname);
    in >> file;
  }

  void parsecontrollerdata() {
    controllerdata_input.sample_time = file["controller"]["sample_time"];

    for (int i = 0; i != m; ++i) {
      std::string str_thruster("thruster");
      str_thruster += std::to_string(i + 1);
      std::string str_type = file[str_thruster]["type"];
      if (str_type == "tunnel") {
        ++thrustallocationdata_input.num_tunnel;
        thrustallocationdata_input.index_thrusters.push_back(1);

        tunnelthrusterdata _tunnelthrusterdata_input;

        // position
        std::vector<double> _position = file[str_thruster]["position"];
        _tunnelthrusterdata_input.lx = _position[0];
        _tunnelthrusterdata_input.ly = _position[1];
        // thrust_contant
        std::vector<double> _thrustconstant =
            file[str_thruster]["thrust_contant"];
        _tunnelthrusterdata_input.K_positive = _thrustconstant[0];
        _tunnelthrusterdata_input.K_negative = _thrustconstant[1];

        // rotation
        _tunnelthrusterdata_input.max_delta_rotation =
            file[str_thruster]["max_delta_rotation"];
        _tunnelthrusterdata_input.max_delta_rotation =
            static_cast<int>(controllerdata_input.sample_time *
                             _tunnelthrusterdata_input.max_delta_rotation);
        _tunnelthrusterdata_input.max_rotation =
            file[str_thruster]["max_rotation"];
        // thrust
        tunnelthrusterdata_input.push_back(_tunnelthrusterdata_input);
      } else if (str_type == "azimuth") {
        ++thrustallocationdata_input.num_azimuth;
        thrustallocationdata_input.index_thrusters.push_back(2);
      } else if (str_type == "rudder") {
        ++thrustallocationdata_input.num_mainrudder;
        thrustallocationdata_input.index_thrusters.push_back(3);
      } else {
        std::cout << "unknow thruster type!\n";
      }
    }
    for (int i = 0; i != 3; ++i)
      std::cout << thrustallocationdata_input.index_thrusters[i] << std::endl;
  }
};

template <int _m, int _n>
std::ostream& operator<<(std::ostream& os, const jsonparse<_m, _n>& _jp) {
  for (unsigned int i = 0; i != _jp.tunnelthrusterdata_input.size(); ++i) {
    std::cout << _jp.tunnelthrusterdata_input[i].lx << std::endl;
    std::cout << _jp.tunnelthrusterdata_input[i].ly << std::endl;
    std::cout << _jp.tunnelthrusterdata_input[i].K_positive << std::endl;
    std::cout << _jp.tunnelthrusterdata_input[i].K_negative << std::endl;
    std::cout << _jp.tunnelthrusterdata_input[i].max_delta_rotation
              << std::endl;
    std::cout << _jp.tunnelthrusterdata_input[i].max_rotation << std::endl;
    std::cout << _jp.tunnelthrusterdata_input[i].max_thrust_positive
              << std::endl;
    std::cout << _jp.tunnelthrusterdata_input[i].max_thrust_negative
              << std::endl;
  }

  return os;
}

#endif /* JSONPARSE_H */
