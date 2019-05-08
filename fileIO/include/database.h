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
#include "estimatordata.h"
#include "gpsdata.h"
#include "plannerdata.h"

class database {
 public:
  explicit database(const std::string &_savepath)
      : db(_savepath),
        clientset({"controller", "estimator", "planner", "GPS"}) {}

  ~database() {}

  void update_gps_table(const gpsRTdata &_gpsRTdata) {
    try {
      std::string str =
          "INSERT INTO GPS"
          "(DATETIME, gps_week, gps_time, heading, pitch, roll, latitude,"
          "longitude, altitude, Ve, Vn, Vu, base_line, NSV1, NSV2,"
          "status, check, UTM_x, UTM_y) VALUES( 0 , julianday('now')";
      convert2string(_gpsRTdata, str);
      str += ");";
      db << str;
    } catch (sqlite::sqlite_exception e) {
      cout << "Unexpected error " << e.what() << endl;
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
    db << "CREATE TABLE IF NOT EXISTS MASTER("
          "CLIENT_ID    INT    PRIMARY KEY    NOT NULL,"
          "TABLE_NAME   TEXT                          ,"
          "DATETIME     TEXT                         );";

    for (int i = 0; i != clientset.size(); ++i) {
      // update clientset
      std::string s_id = std::to_string(i);
      // create string for sqlite
      std::string str(
          "INSERT INTO MASTER (CLIENT_ID,TABLE_NAME,DATETIME) VALUES(");
      str += s_id;
      str += ", ";
      str += clientset[i];
      str += "', DATETIME('now'));";
      db << str;
    }
  }

  // create GPS table
  void create_GPS_table() {
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
        " status      TEXT, "
        " check       TEXT, "
        " UTM_x       DOUBLE, "
        " UTM_y       DOUBLE);";

    db << str;
  }
  // create each controller table (TODO: controller for underactuated USV)
  void create_controller_table(int m) {
    // real-time data in the controller
    std::string str =
        "CREATE TABLE controller"
        "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
        " DATETIME    TEXT       NOT NULL,"
        " tauX        DOUBLE, "
        " tauY        DOUBLE, "
        " tauMz       DOUBLE, "; /* tau: desired force */

    // the angle of each propeller
    for (int i = 0; i != m; ++i) {
      str += " alpha" + std::to_string(i + 1) + " INT,";
    }
    // the speed of each propeller
    for (int i = 0; i != m; ++i) {
      str += " rpm" + std::to_string(i + 1) + " INT,";
    }
    // BalphaU: estimated force
    str +=
        " estX        DOUBLE, "
        " estY        DOUBLE, "
        " estMz       DOUBLE);";

    db << str;
  }

  // create each estimator table
  void create_estimator_table() {
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
  }

  // create planner table (TODO)
  void create_planner_table() {}

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
    _str += ", ";
    _str += std::string(1, _gpsRTdata.status);
    _str += ", ";
    _str += std::string(check);
    _str += ", ";
    _str += std::to_string(UTM_x);
    _str += ", ";
    _str += std::to_string(UTM_y);
  }
  void convert2string(const estimatorRTdata &_RTdata, std::string &_str) {}
};
#endif /* _DATABASE_H_ */