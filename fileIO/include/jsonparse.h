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
#include <cmath>
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
  explicit jsonparse(const std::string& _jsonname) : jsonname(_jsonname) {
    readjson();
  }
  jsonparse() = delete;
  ~jsonparse() {}

  vessel getvessel() const noexcept { return vesseldata_input; }
  controllerdata getcontrollerdata() const noexcept {
    return controllerdata_input;
  }
  thrustallocationdata getthrustallocationdata() const noexcept {
    return thrustallocationdata_input;
  }
  std::vector<tunnelthrusterdata> gettunneldata() const noexcept {
    return tunnelthrusterdata_input;
  }
  std::vector<azimuththrusterdata> getazimuthdata() const noexcept {
    return azimuththrusterdata_input;
  }
  std::vector<ruddermaindata> getmainrudderdata() const noexcept {
    return ruddermaindata_input;
  }
  std::vector<pidcontrollerdata> getpiddata() const noexcept {
    return pidcontrollerdata_input;
  }
  estimatordata getestimatordata() const noexcept {
    return estimatordata_input;
  }
  plannerdata getplannerdata() const noexcept { return plannerdata_input; }

  std::string getsqlitedata() const noexcept { return dbpath; }
  std::string getgpsport() const noexcept { return gps_port; }
  std::string getguiport() const noexcept { return gui_port; }
  std::string getremotecontrolport() const noexcept { return rc_port; }
  unsigned long getgpsbaudrate() const noexcept { return gps_baudrate; }
  unsigned long getguibaudrate() const noexcept { return gui_baudrate; }
  unsigned long getrcbaudrate() const noexcept { return rc_baudrate; }

  void readjson() {
    parsejson();
    parsevesselpropertydata();
    parseplannerdata();
    parsecontrollerdata();
    parseestimatordata();
    parsesqlitedata();
    paresecomcenter();
  }

 private:
  std::string jsonname;
  // json file = json::parse(in);
  nlohmann::json file;

  utilityio _utilityio;

  std::string dbpath;  // directory for database file

  unsigned long gps_baudrate;
  std::string gps_port;
  unsigned long gui_baudrate;
  std::string gui_port;
  unsigned long rc_baudrate;
  std::string rc_port;
  // vessel property
  vessel vesseldata_input{
      Eigen::Matrix3d::Zero(),  // Mass
      Eigen::Matrix3d::Zero(),  // AddedMass
      Eigen::Matrix3d::Zero(),  // Damping
      Eigen::Vector2d::Zero(),  // cog
      Eigen::Vector2d::Zero(),  // x_thrust
      Eigen::Vector2d::Zero(),  // y_thrust
      Eigen::Vector2d::Zero(),  // mz_thrust
      Eigen::Vector2d::Zero(),  // surge_v
      Eigen::Vector2d::Zero(),  // sway_v
      Eigen::Vector2d::Zero(),  // yaw_v
      Eigen::Vector2d::Zero(),  // roll_v
      0,                        // L
      0                         // B
  };

  // controllerdata
  controllerdata controllerdata_input{
      0.1,           // sample_time
      MANUAL,        // controlmode
      WINDON,        // windstatus
      FULLYACTUATED  // index_actuation
  };
  // plannerdata
  plannerdata plannerdata_input{
      0.1,  // sample_time
      0,    // los_radius
      0,    // los_capture_radius
  };
  // thrustallocationdata
  thrustallocationdata thrustallocationdata_input{
      0,  // num_tunnel
      0,  // num_azimuth
      0,  // num_mainrudder
      {}  // index_thrusters
  };
  std::vector<tunnelthrusterdata> tunnelthrusterdata_input;
  std::vector<azimuththrusterdata> azimuththrusterdata_input;
  std::vector<ruddermaindata> ruddermaindata_input;

  std::vector<pidcontrollerdata> pidcontrollerdata_input;

  // estimatordata
  estimatordata estimatordata_input{
      0.1,  // sample_time
  };

  void parsejson() {
    // read a JSON file
    std::ifstream in(jsonname);
    in >> file;
  }

  void parsecontrollerdata() {
    // controller
    controllerdata_input.sample_time =
        file["controller"]["sample_time"].get<double>();
    pidcontrollerdata _pidcontrollerdata_input;
    // surge-- controller
    _pidcontrollerdata_input.position_P =
        file["controller"]["surge"]["position_P"].get<double>();
    _pidcontrollerdata_input.position_I =
        file["controller"]["surge"]["position_I"].get<double>();
    _pidcontrollerdata_input.velocity_P =
        file["controller"]["surge"]["velocity_P"].get<double>();
    _pidcontrollerdata_input.velocity_I =
        file["controller"]["surge"]["velocity_I"].get<double>();

    _pidcontrollerdata_input.position_allowed_error =
        file["controller"]["surge"]["position_allowed_error"].get<double>();
    _pidcontrollerdata_input.velocity_allowed_error =
        file["controller"]["surge"]["velocity_allowed_error"].get<double>();
    _pidcontrollerdata_input.min_output =
        file["controller"]["surge"]["min_output"].get<double>();
    _pidcontrollerdata_input.max_output =
        file["controller"]["surge"]["max_output"].get<double>();
    pidcontrollerdata_input.push_back(_pidcontrollerdata_input);

    // sway-- controller
    _pidcontrollerdata_input.position_P =
        file["controller"]["sway"]["position_P"].get<double>();
    _pidcontrollerdata_input.position_I =
        file["controller"]["sway"]["position_I"].get<double>();
    _pidcontrollerdata_input.velocity_P =
        file["controller"]["sway"]["velocity_P"].get<double>();
    _pidcontrollerdata_input.velocity_I =
        file["controller"]["sway"]["velocity_I"].get<double>();

    _pidcontrollerdata_input.position_allowed_error =
        file["controller"]["sway"]["position_allowed_error"].get<double>();
    _pidcontrollerdata_input.velocity_allowed_error =
        file["controller"]["sway"]["velocity_allowed_error"].get<double>();

    _pidcontrollerdata_input.min_output =
        file["controller"]["sway"]["min_output"].get<double>();
    _pidcontrollerdata_input.max_output =
        file["controller"]["sway"]["max_output"].get<double>();
    pidcontrollerdata_input.push_back(_pidcontrollerdata_input);

    // yaw-- controller
    _pidcontrollerdata_input.position_P =
        file["controller"]["yaw"]["position_P"].get<double>();
    _pidcontrollerdata_input.position_I =
        file["controller"]["yaw"]["position_I"].get<double>();
    _pidcontrollerdata_input.velocity_P =
        file["controller"]["yaw"]["velocity_P"].get<double>();
    _pidcontrollerdata_input.velocity_I =
        file["controller"]["yaw"]["velocity_I"].get<double>();

    _pidcontrollerdata_input.position_allowed_error =
        file["controller"]["yaw"]["position_allowed_error"].get<double>();
    _pidcontrollerdata_input.velocity_allowed_error =
        file["controller"]["yaw"]["velocity_allowed_error"].get<double>();

    _pidcontrollerdata_input.min_output =
        file["controller"]["yaw"]["min_output"].get<double>();
    _pidcontrollerdata_input.max_output =
        file["controller"]["yaw"]["max_output"].get<double>();
    pidcontrollerdata_input.push_back(_pidcontrollerdata_input);
    // thrusters
    for (int i = 0; i != m; ++i) {
      std::string str_thruster("thruster");
      str_thruster += std::to_string(i + 1);
      std::string str_type = file[str_thruster]["type"];
      if (str_type == "tunnel") {
        // update # of tunnels and index_thruster
        ++thrustallocationdata_input.num_tunnel;
        thrustallocationdata_input.index_thrusters.push_back(1);

        tunnelthrusterdata _thrusterdata_input;

        // position
        std::vector<double> _position = file[str_thruster]["position"];
        _thrusterdata_input.lx = _position[0];
        _thrusterdata_input.ly = _position[1];
        // thrust_constant
        std::vector<double> _thrustconstant =
            file[str_thruster]["thrust_constant"];
        _thrusterdata_input.K_positive = _thrustconstant[0];
        _thrusterdata_input.K_negative = _thrustconstant[1];

        // rotation
        _thrusterdata_input.max_delta_rotation = static_cast<int>(
            std::round(controllerdata_input.sample_time *
                       file[str_thruster]["max_delta_rotation"].get<double>()));
        _thrusterdata_input.max_rotation =
            file[str_thruster]["max_rotation"].get<int>();
        // thrust
        _thrusterdata_input.max_thrust_positive =
            _thrusterdata_input.K_positive *
            std::pow(_thrusterdata_input.max_rotation, 2);
        _thrusterdata_input.max_thrust_negative =
            _thrusterdata_input.K_negative *
            std::pow(_thrusterdata_input.max_rotation, 2);

        //
        tunnelthrusterdata_input.push_back(_thrusterdata_input);
      } else if (str_type == "azimuth") {
        // update # of azimuth and index_thruster
        ++thrustallocationdata_input.num_azimuth;
        thrustallocationdata_input.index_thrusters.push_back(2);

        azimuththrusterdata _thrusterdata_input;

        // position
        std::vector<double> _position = file[str_thruster]["position"];
        _thrusterdata_input.lx = _position[0];
        _thrusterdata_input.ly = _position[1];
        // thrust_constant
        _thrusterdata_input.K =
            file[str_thruster]["thrust_constant"].get<double>();

        // rotation
        _thrusterdata_input.max_delta_rotation = static_cast<int>(
            std::round(controllerdata_input.sample_time *
                       file[str_thruster]["max_delta_rotation"].get<double>()));
        _thrusterdata_input.max_rotation =
            file[str_thruster]["max_rotation"].get<int>();
        _thrusterdata_input.min_rotation =
            file[str_thruster]["min_rotation"].get<int>();

        // alpha
        _thrusterdata_input.max_delta_alpha =
            controllerdata_input.sample_time *
            file[str_thruster]["max_delta_alpha"].get<double>();

        _thrusterdata_input.max_alpha =
            file[str_thruster]["max_alpha"].get<double>();
        _thrusterdata_input.min_alpha =
            file[str_thruster]["min_alpha"].get<double>();
        // thrust
        _thrusterdata_input.max_thrust =
            _thrusterdata_input.K *
            std::pow(_thrusterdata_input.max_rotation, 2);
        _thrusterdata_input.min_thrust =
            _thrusterdata_input.K *
            std::pow(_thrusterdata_input.min_rotation, 2);

        //
        azimuththrusterdata_input.push_back(_thrusterdata_input);
      } else if (str_type == "rudder") {
        ++thrustallocationdata_input.num_mainrudder;
        thrustallocationdata_input.index_thrusters.push_back(3);

        ruddermaindata _thrusterdata_input;

        // position
        std::vector<double> _position = file[str_thruster]["position"];
        _thrusterdata_input.lx = _position[0];
        _thrusterdata_input.ly = _position[1];
        // thrust_constant
        _thrusterdata_input.K =
            file[str_thruster]["thrust_constant"].get<double>();
        _thrusterdata_input.Cy =
            file[str_thruster]["rudder_constant"].get<double>();
        // rotation
        _thrusterdata_input.max_delta_rotation = static_cast<int>(
            std::round(controllerdata_input.sample_time *
                       file[str_thruster]["max_delta_rotation"].get<double>()));
        _thrusterdata_input.max_rotation =
            file[str_thruster]["max_rotation"].get<int>();
        _thrusterdata_input.min_rotation =
            file[str_thruster]["min_rotation"].get<int>();

        // varphi
        _thrusterdata_input.max_delta_varphi = static_cast<int>(
            std::round(controllerdata_input.sample_time *
                       file[str_thruster]["max_delta_varphi"].get<double>()));

        _thrusterdata_input.max_varphi =
            file[str_thruster]["max_varphi"].get<int>();
        _thrusterdata_input.min_varphi =
            file[str_thruster]["min_varphi"].get<int>();
        // thrust
        _thrusterdata_input.max_thrust =
            _thrusterdata_input.K *
            std::pow(_thrusterdata_input.max_rotation, 2);
        _thrusterdata_input.min_thrust =
            _thrusterdata_input.K *
            std::pow(_thrusterdata_input.min_rotation, 2);

        //
        ruddermaindata_input.push_back(_thrusterdata_input);

      } else {
        std::cout << "unknow thruster type!\n";
      }
    }

  }  // parsecontrollerdata()

  void parseestimatordata() {
    estimatordata_input.sample_time =
        file["estimator"]["sample_time"].get<double>();
    // bool kalman_on = file["estimator"]["KALMANON"];
    // if (kalman_on)
    //   estimatordata_input.kalman_use = KALMANON;
    // else
    //   estimatordata_input.kalman_use = KALMANOFF;
  }  // parseestimatordata
  void parseplannerdata() {
    plannerdata_input.sample_time =
        file["planner"]["sample_time"].get<double>();
    plannerdata_input.los_radius =
        vesseldata_input.L *
        file["planner"]["LOS"]["los_radius_co"].get<double>();
    plannerdata_input.los_capture_radius =
        vesseldata_input.L *
        file["planner"]["LOS"]["capture_radius_co"].get<double>();
  }  // parseplannerdata
  void parsevesselpropertydata() {
    vesseldata_input.Mass = _utilityio.convertstdvector2EigenMat<double, 3, 3>(
        file["property"]["Mass"].get<std::vector<double>>());
    vesseldata_input.AddedMass =
        _utilityio.convertstdvector2EigenMat<double, 3, 3>(
            file["property"]["AddedMass"].get<std::vector<double>>());
    vesseldata_input.Damping =
        _utilityio.convertstdvector2EigenMat<double, 3, 3>(
            file["property"]["Damping"].get<std::vector<double>>());
    vesseldata_input.cog = _utilityio.convertstdvector2EigenMat<double, 2, 1>(
        file["property"]["CoG"].get<std::vector<double>>());

    vesseldata_input.x_thrust
        << file["controller"]["surge"]["min_output"].get<double>(),
        file["controller"]["surge"]["max_output"].get<double>();
    vesseldata_input.y_thrust
        << file["controller"]["sway"]["min_output"].get<double>(),
        file["controller"]["sway"]["max_output"].get<double>();
    vesseldata_input.mz_thrust
        << file["controller"]["yaw"]["min_output"].get<double>(),
        file["controller"]["yaw"]["max_output"].get<double>();

    vesseldata_input
        .surge_v = _utilityio.convertstdvector2EigenMat<double, 2, 1>(
        file["property"]["velocity_limit"]["surge"].get<std::vector<double>>());
    vesseldata_input
        .sway_v = _utilityio.convertstdvector2EigenMat<double, 2, 1>(
        file["property"]["velocity_limit"]["sway"].get<std::vector<double>>());
    vesseldata_input.yaw_v = _utilityio.convertstdvector2EigenMat<double, 2, 1>(
        file["property"]["velocity_limit"]["yaw"].get<std::vector<double>>());
    vesseldata_input
        .roll_v = _utilityio.convertstdvector2EigenMat<double, 2, 1>(
        file["property"]["velocity_limit"]["roll"].get<std::vector<double>>());

    vesseldata_input.L = file["property"]["L"].get<double>();
    vesseldata_input.B = file["property"]["B"].get<double>();

  }  // parsevesselpropertydata

  void parsesqlitedata() {
    dbpath = file["project_directory"].get<std::string>() +
             file["dbpath"].get<std::string>();

  }  // parsesqlitedata

  void paresecomcenter() {
    gps_port = file["comcenter"]["GPS"]["port"];
    gps_baudrate = file["comcenter"]["GPS"]["baudrate"].get<unsigned long>();
    gui_port = file["comcenter"]["GUI_server"]["port"];
    gui_baudrate =
        file["comcenter"]["GUI_server"]["baudrate"].get<unsigned long>();
    rc_port = file["comcenter"]["remotecontrol"]["port"];
    rc_baudrate =
        file["comcenter"]["remotecontrol"]["baudrate"].get<unsigned long>();
  }
};

template <int _m, int _n>
std::ostream& operator<<(std::ostream& os, const jsonparse<_m, _n>& _jp) {
  os << "index of each thruster:\n";
  for (int i = 0; i != _m; ++i)
    os << _jp.thrustallocationdata_input.index_thrusters[i] << " ";

  os << "info of each tunnel thruster:\n";
  for (unsigned int i = 0; i != _jp.tunnelthrusterdata_input.size(); ++i) {
    os << _jp.tunnelthrusterdata_input[i].lx << std::endl;
    os << _jp.tunnelthrusterdata_input[i].ly << std::endl;
    os << _jp.tunnelthrusterdata_input[i].K_positive << std::endl;
    os << _jp.tunnelthrusterdata_input[i].K_negative << std::endl;
    os << _jp.tunnelthrusterdata_input[i].max_delta_rotation << std::endl;
    os << _jp.tunnelthrusterdata_input[i].max_rotation << std::endl;
    os << _jp.tunnelthrusterdata_input[i].max_thrust_positive << std::endl;
    os << _jp.tunnelthrusterdata_input[i].max_thrust_negative << std::endl;
  }

  os << "info of each azimuth thruster:\n";
  for (unsigned int i = 0; i != _jp.azimuththrusterdata_input.size(); ++i) {
    os << _jp.azimuththrusterdata_input[i].lx << std::endl;
    os << _jp.azimuththrusterdata_input[i].ly << std::endl;
    os << _jp.azimuththrusterdata_input[i].K << std::endl;
    os << _jp.azimuththrusterdata_input[i].max_delta_rotation << std::endl;
    os << _jp.azimuththrusterdata_input[i].max_rotation << std::endl;
    os << _jp.azimuththrusterdata_input[i].min_rotation << std::endl;
    os << _jp.azimuththrusterdata_input[i].max_delta_alpha << std::endl;
    os << _jp.azimuththrusterdata_input[i].max_alpha << std::endl;
    os << _jp.azimuththrusterdata_input[i].min_alpha << std::endl;
    os << _jp.azimuththrusterdata_input[i].max_thrust << std::endl;
    os << _jp.azimuththrusterdata_input[i].min_thrust << std::endl;
  }

  os << "info of each main thruster with rudder:\n";
  for (unsigned int i = 0; i != _jp.ruddermaindata_input.size(); ++i) {
    os << _jp.ruddermaindata_input[i].lx << std::endl;
    os << _jp.ruddermaindata_input[i].ly << std::endl;
    os << _jp.ruddermaindata_input[i].K << std::endl;
    os << _jp.ruddermaindata_input[i].Cy << std::endl;
    os << _jp.ruddermaindata_input[i].max_delta_rotation << std::endl;
    os << _jp.ruddermaindata_input[i].max_rotation << std::endl;
    os << _jp.ruddermaindata_input[i].min_rotation << std::endl;
    os << _jp.ruddermaindata_input[i].max_delta_varphi << std::endl;
    os << _jp.ruddermaindata_input[i].max_varphi << std::endl;
    os << _jp.ruddermaindata_input[i].min_varphi << std::endl;
  }

  os << "pid controller:\n";
  for (unsigned int i = 0; i != _jp.pidcontrollerdata_input.size(); ++i) {
    os << _jp.pidcontrollerdata_input[i].position_P << std::endl;
    os << _jp.pidcontrollerdata_input[i].position_I << std::endl;
    os << _jp.pidcontrollerdata_input[i].velocity_P << std::endl;
    os << _jp.pidcontrollerdata_input[i].velocity_I << std::endl;
    os << _jp.pidcontrollerdata_input[i].position_allowed_error << std::endl;
    os << _jp.pidcontrollerdata_input[i].velocity_allowed_error << std::endl;
    os << _jp.pidcontrollerdata_input[i].min_output << std::endl;
    os << _jp.pidcontrollerdata_input[i].max_output << std::endl;
  }
  os << "estimator:\n";
  os << _jp.estimatordata_input.sample_time << std::endl;
  os << "planner:\n";
  os << _jp.plannerdata_input.sample_time << std::endl;
  os << _jp.plannerdata_input.los_radius << std::endl;
  os << _jp.plannerdata_input.los_capture_radius << std::endl;

  os << "Mass property:\n";
  os << _jp.vesseldata_input.Mass << std::endl;
  os << _jp.vesseldata_input.AddedMass << std::endl;
  os << _jp.vesseldata_input.Damping << std::endl;
  os << _jp.vesseldata_input.cog << std::endl;
  os << _jp.vesseldata_input.x_thrust << std::endl;
  os << _jp.vesseldata_input.y_thrust << std::endl;
  os << _jp.vesseldata_input.mz_thrust << std::endl;
  os << _jp.vesseldata_input.surge_v << std::endl;
  os << _jp.vesseldata_input.sway_v << std::endl;
  os << _jp.vesseldata_input.yaw_v << std::endl;
  os << _jp.vesseldata_input.roll_v << std::endl;

  os << "comcenter:\n";
  os << _jp.gps_port << " " << _jp.gps_baudrate << std::endl;
  os << _jp.gui_port << " " << _jp.gui_baudrate << std::endl;
  os << _jp.rc_port << " " << _jp.rc_baudrate << std::endl;

  return os;
}

#endif /* JSONPARSE_H */
