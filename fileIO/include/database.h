/*
***********************************************************************
* database.h:
* database using sqlite3 and sqlite modern cpp wrapper
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _DATABASE_H_
#define _DATABASE_H_

#include <sqlite_modern_cpp.h>
#include <Eigen/Core>
#include <string>
#include <vector>
#include "controllerdata.h"
#include "easylogging++.h"
#include "estimatordata.h"
#include "gpsdata.h"
#include "plannerdata.h"

template <int m, int n = 3>
class database {
 public:
  explicit database(const std::string &_savepath)
      : db(_savepath),
        clientset(
            {"controller", "estimator", "planner", "GPS", "wind", "motor"}) {}

  ~database() {}

  void initializetables() {
    create_mastertable();
    create_GPS_table();
    create_controller_table();
    create_estimator_table();
    create_planner_table();
  }
  // insert a bow into gps table
  void update_gps_table(const gpsRTdata &_gpsRTdata) {
    try {
      std::string str =
          "INSERT INTO GPS"
          "(DATETIME, gps_week, gps_time, heading, pitch, roll, latitude,"
          "longitude, altitude, Ve, Vn, Vu, base_line, NSV1, NSV2,"
          "gpsstatus, gpscheck, UTM_x, UTM_y) VALUES(julianday('now')";
      convert2string(_gpsRTdata, str);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }
  // insert a bow into controller table
  void update_controller_table(const controllerRTdata<m, n> &_RTdata) {
    try {
      std::string str =
          "INSERT INTO controller"
          "(DATETIME ";

      // tau: desired force
      for (int i = 0; i != n; ++i) {
        str += ", tau" + std::to_string(i + 1);
      }
      // the angle of each propeller
      for (int i = 0; i != m; ++i) {
        str += ", alpha" + std::to_string(i + 1);
      }
      // the speed of each propeller
      for (int i = 0; i != m; ++i) {
        str += ", rpm" + std::to_string(i + 1);
      }
      // BalphaU: estimated force
      for (int i = 0; i != n; ++i) {
        str += ", est" + std::to_string(i + 1);
      }

      str += ") VALUES(julianday('now')";
      convert2string(_RTdata, str);
      str += ");";

      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-controller") << e.what();
    }
  }
  // insert a bow into estimator table
  void update_estimator_table(const estimatorRTdata &_RTdata) {
    try {
      std::string str =
          "INSERT INTO estimator"
          "(DATETIME, meas_x, meas_y, meas_theta, meas_u, meas_v, meas_r,"
          "state_x, state_y, state_theta, state_u, state_v, state_r, "
          "perror_x, perror_y, perror_mz, verror_x, verror_y, verror_mz"
          ") VALUES(julianday('now')";
      convert2string(_RTdata, str);
      str += ");";
      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-estimator") << e.what();
    }
  }
  // insert a bow into estimator table
  void update_planner_table(const plannerRTdata &_RTdata) {
    try {
      std::string str =
          "INSERT INTO planner"
          "(DATETIME, set_x, set_y, set_theta, set_u, set_v, set_r,"
          "command_x, command_y, command_theta, wp0_x, wp0_y, wp1_x, wp1_y) "
          "VALUES(julianday('now')";
      convert2string(_RTdata, str);
      str += ");";
      db << str;
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql-planner") << e.what();
    }
  }

 private:
  sqlite::database db;
  // the set of table names
  // 0 --> controller
  // 1 --> estimator
  // 2 --> planner
  // 3 --> GPS
  std::vector<std::string> clientset;

  // create master table about the info of each tables
  void create_mastertable() {
    try {
      db << "CREATE TABLE IF NOT EXISTS MASTER("
            "CLIENT_ID    INT    PRIMARY KEY    NOT NULL,"
            "TABLE_NAME   TEXT                          ,"
            "DATETIME     TEXT                         );";

      for (unsigned int i = 0; i != clientset.size(); ++i) {
        // update clientset
        std::string s_id = std::to_string(i);
        // create string for sqlite
        std::string str(
            "INSERT INTO MASTER (CLIENT_ID,TABLE_NAME,DATETIME) VALUES(");
        str += s_id;
        str += ", '";
        str += clientset[i];
        str += "' , DATETIME('now'));";
        db << str;
      }
    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }

  // create GPS table
  void create_GPS_table() {
    try {
      // GPS/IMU sensor GPFPD format
      std::string str =
          "CREATE TABLE GPS"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,"
          " gps_week    INT, "
          " gps_time    DOUBLE, "
          " heading     DOUBLE, "
          " pitch       DOUBLE, "
          " roll        DOUBLE, "
          " latitude    DOUBLE, "
          " longitude   DOUBLE, "
          " altitude    DOUBLE, "
          " Ve          DOUBLE, "
          " Vn          DOUBLE, "
          " Vu          DOUBLE, "
          " base_line   DOUBLE, "
          " NSV1        INT, "
          " NSV2        INT, "
          " gpsstatus   TEXT, "
          " gpscheck    TEXT,"
          " UTM_x       DOUBLE, "
          " UTM_y       DOUBLE);";
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }
  // create each controller table (TODO: controller for underactuated USV)
  void create_controller_table() {
    try {
      // real-time data in the controller
      std::string str =
          "CREATE TABLE controller"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL";

      // tau: desired force
      for (int i = 0; i != n; ++i) {
        str += " ,tau" + std::to_string(i + 1) + " DOUBLE";
      }
      // the angle of each propeller
      for (int i = 0; i != m; ++i) {
        str += " ,alpha" + std::to_string(i + 1) + " INT";
      }
      // the speed of each propeller
      for (int i = 0; i != m; ++i) {
        str += " ,rpm" + std::to_string(i + 1) + " INT";
      }
      // BalphaU: estimated force
      for (int i = 0; i != n; ++i) {
        str += " ,est" + std::to_string(i + 1) + " DOUBLE";
      }
      str += ");";
      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }

  // create each estimator table
  void create_estimator_table() {
    try {
      // real-time data in the estimator
      std::string str =
          "CREATE TABLE estimator"
          "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME    TEXT       NOT NULL,"
          " meas_x      DOUBLE, "
          " meas_y      DOUBLE, "
          " meas_theta  DOUBLE, "
          " meas_u      DOUBLE, "
          " meas_v      DOUBLE, "
          " meas_r      DOUBLE, " /* Measurement */
          " state_x     DOUBLE, "
          " state_y     DOUBLE, "
          " state_theta DOUBLE, "
          " state_u     DOUBLE, "
          " state_v     DOUBLE, "
          " state_r     DOUBLE, " /* state */
          " perror_x    DOUBLE, "
          " perror_y    DOUBLE, "
          " perror_mz   DOUBLE, " /* perror */
          " verror_x    DOUBLE, "
          " verror_y    DOUBLE, "
          " verror_mz   DOUBLE); "; /* verror */

      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }

  // create planner table
  void create_planner_table() {
    try {
      // real-time data in the estimator
      std::string str =
          "CREATE TABLE planner"
          "(ID            INTEGER PRIMARY KEY AUTOINCREMENT,"
          " DATETIME      TEXT       NOT NULL,"
          " set_x         DOUBLE, "
          " set_y         DOUBLE, "
          " set_theta     DOUBLE, " /* setpoint */
          " set_u         DOUBLE, "
          " set_v         DOUBLE, "
          " set_r         DOUBLE, " /* v_setpoint */
          " command_x     DOUBLE, "
          " command_y     DOUBLE, "
          " command_theta DOUBLE, " /* command */
          " wp0_x         DOUBLE, "
          " wp0_y         DOUBLE, "
          " wp1_x         DOUBLE, "
          " wp1_y         DOUBLE);"; /* waypoints */

      db << str;

    } catch (sqlite::sqlite_exception &e) {
      CLOG(ERROR, "sql") << e.what();
    }
  }

  // convert real time GPS data to sql string
  void convert2string(const gpsRTdata &_gpsRTdata, std::string &_str) {
    _str += ", ";
    _str += std::to_string(_gpsRTdata.date);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.time);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.heading);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.pitch);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.roll);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.latitude);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.longitude);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.altitude);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.Ve);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.Vn);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.Vu);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.base_line);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.NSV1);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.NSV2);
    _str += ", '";
    _str += std::string(1, _gpsRTdata.status);
    _str += "' , '";
    _str += std::string(_gpsRTdata.check);
    _str += "' , ";
    _str += std::to_string(_gpsRTdata.UTM_x);
    _str += ", ";
    _str += std::to_string(_gpsRTdata.UTM_y);
  }
  void convert2string(const controllerRTdata<m, n> &_RTdata,
                      std::string &_str) {
    // tau: desired force
    for (int i = 0; i != n; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.tau(i));
    }
    // the angle of each propeller
    for (int i = 0; i != m; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.alpha_deg(i));
    }
    // the speed of each propeller
    for (int i = 0; i != m; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.rotation(i));
    }
    // BalphaU: estimated force
    for (int i = 0; i != n; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.BalphaU(i));
    }
  }
  void convert2string(const estimatorRTdata &_RTdata, std::string &_str) {
    // Measurement
    for (int i = 0; i != 6; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.Measurement(i));
    }
    // State
    for (int i = 0; i != 6; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.State(i));
    }
    // Perror
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.p_error(i));
    }
    // verror
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.v_error(i));
    }
  }

  void convert2string(const plannerRTdata &_RTdata, std::string &_str) {
    // setpoint
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.setpoint(i));
    }
    // v_setpoint
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.v_setpoint(i));
    }
    // command
    for (int i = 0; i != 3; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.command(i));
    }
    // waypoint0
    for (int i = 0; i != 2; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.waypoint0(i));
    }
    // waypoint1
    for (int i = 0; i != 2; ++i) {
      _str += ", ";
      _str += std::to_string(_RTdata.waypoint1(i));
    }
  }
};
#endif /* _DATABASE_H_ */