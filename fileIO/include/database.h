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
};
#endif /* _DATABASE_H_ */