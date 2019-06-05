/*
***********************************************************************
* guiclient.h: communication for gui client
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _GUICLIENT_H_
#define _GUICLIENT_H_

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "serial/serial.h"
#include "timecounter.h"

struct indicators {
  // indicator for gui connection: 0 --> disconnect, 1 -->connect
  int gui_connection;
  // indicator for joystick connection: 0 --> disconnect, 1 -->connect
  int joystick_connection;
  int indicator_controlmode;
  int indicator_windstatus;
};

// real time data in planner
struct plannerRTdata {};

struct motorRTdata {
  float command_alpha[6];
  float command_rotation[6];
  int feedback_alpha[6];
  int feedback_rotation[6];
  int feedback_torque[12];
  int feedback_info[36];  // run/warning/alarm
  char feedback_allinfo;  // 总的报警 / 复位信息
};

struct gpsRTdata {
  double latitude;   // 纬度 -90 ~ 90
  double longitude;  // 经度 -180 ~ 180
};

struct estimatorRTdata {
  double x;
  double y;
  double heading;
  double roll;
  double pitch;
  double u;
  double v;
  double r;
};

class guiclient {
 public:
  guiclient()
      : my_serial("/dev/ttyS4", 19200, serial::Timeout::simpleTimeout(500)),
        send_buffer(""),
        recv_buffer(""),
        gui_connection_count(0) {
    enumerate_ports();
    judgeserialstatus();
  }
  ~guiclient() {}

  void guicommunication() {
    timecounter _timer;
    senddata2vessel();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(500 - _timer.timeelapsed()));
    parsedatafromvessel();
    std::cout << "send message:" << send_buffer << std::endl;
    std::cout << "recv message:" << recv_buffer << std::endl;
  }

 private:
  serial::Serial my_serial;
  std::string send_buffer;
  std::string recv_buffer;
  int gui_connection_count;
  indicators _indicators{
      0,  // gui_connection
      0,  // joystick_connection
      0,  // indicator_controlmode
      0   // indicator_windstatus
  };
  motorRTdata _motorRTdata;
  gpsRTdata _gpsRTdata{
      0.0,  // latitude
      0.0   // longitude
  };

  estimatorRTdata _estimatorRTdata{
      0,  // x
      0,  // y
      0,  // heading
      0,  // roll
      0,  // pitch
      0,  // u
      0,  // v
      0   // r;
  };

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
      std::cout << device.port.c_str() << ", " << device.description.c_str()
                << ", " << device.hardware_id.c_str() << std::endl;
    }
  }

  void judgeserialstatus() {
    if (my_serial.isOpen())
      std::cout << " serial port open successful!\n";
    else
      std::cout << " serial port open failure!\n";
  }

  void parsedatafromvessel() {
    recv_buffer = my_serial.readline(400, "\n");

    std::size_t pos = recv_buffer.find("$IPAC");
    if (pos != std::string::npos) {
      recv_buffer = recv_buffer.substr(pos);
      sscanf(recv_buffer.c_str(),
             "$IPAC"
             ",%d,%d,%d,%d"                      // indicator
             ",%lf,%lf"                          // gps
             ",%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf"  // estimator
             ",%f,%f,%f,%f,%f,%f"                // motor - command_alpha
             ",%f,%f,%f,%f,%f,%f"                // motor - command_rotation
             ",%d,%d,%d,%d,%d,%d"                // motor - feedback_alpha
             ",%d,%d,%d,%d,%d,%d"                // motor - feedback_rotation
             ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"  // motor - feedback_torque
             ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"  // motor - feedback_info
             ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"  // motor - feedback_info
             ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"  // motor - feedback_info
             ,
             &_indicators.gui_connection,         // indicator
             &_indicators.joystick_connection,    //
             &_indicators.indicator_controlmode,  //
             &_indicators.indicator_windstatus,   // indicator
             &_gpsRTdata.latitude,                // gps
             &_gpsRTdata.longitude,               // gps
             &_estimatorRTdata.x,                 // estimator
             &_estimatorRTdata.y,                 //
             &_estimatorRTdata.heading,           //
             &_estimatorRTdata.u,                 //
             &_estimatorRTdata.v,                 //
             &_estimatorRTdata.r,                 //
             &_estimatorRTdata.roll,              //
             &_estimatorRTdata.pitch,             // estimator
             &_motorRTdata.command_alpha[0],      // motor-command_alpha
             &_motorRTdata.command_alpha[1],      //
             &_motorRTdata.command_alpha[2],      //
             &_motorRTdata.command_alpha[3],      //
             &_motorRTdata.command_alpha[4],      //
             &_motorRTdata.command_alpha[5],      // motor-command_alpha
             &_motorRTdata.command_rotation[0],   // motor-command_rotation
             &_motorRTdata.command_rotation[1],   //
             &_motorRTdata.command_rotation[2],   //
             &_motorRTdata.command_rotation[3],   //
             &_motorRTdata.command_rotation[4],   //
             &_motorRTdata.command_rotation[5],   // motor-command_rotation
             &_motorRTdata.feedback_alpha[0],     // motor-feedback_alpha
             &_motorRTdata.feedback_alpha[1],     //
             &_motorRTdata.feedback_alpha[2],     //
             &_motorRTdata.feedback_alpha[3],     //
             &_motorRTdata.feedback_alpha[4],     //
             &_motorRTdata.feedback_alpha[5],     // motor-feedback_alpha
             &_motorRTdata.feedback_rotation[0],  // motor-feedback_rotation
             &_motorRTdata.feedback_rotation[1],  //
             &_motorRTdata.feedback_rotation[2],  //
             &_motorRTdata.feedback_rotation[3],  //
             &_motorRTdata.feedback_rotation[4],  //
             &_motorRTdata.feedback_rotation[5],  // motor-feedback_rotation
             &_motorRTdata.feedback_torque[0],    // motor-feedback_torque
             &_motorRTdata.feedback_torque[1],    //
             &_motorRTdata.feedback_torque[2],    //
             &_motorRTdata.feedback_torque[3],    //
             &_motorRTdata.feedback_torque[4],    //
             &_motorRTdata.feedback_torque[5],    //
             &_motorRTdata.feedback_torque[6],    //
             &_motorRTdata.feedback_torque[7],    //
             &_motorRTdata.feedback_torque[8],    //
             &_motorRTdata.feedback_torque[9],    //
             &_motorRTdata.feedback_torque[10],   //
             &_motorRTdata.feedback_torque[11],   // motor-feedback_torque
             &_motorRTdata.feedback_info[0],      // motor-feedback_info
             &_motorRTdata.feedback_info[1],      //
             &_motorRTdata.feedback_info[2],      //
             &_motorRTdata.feedback_info[3],      //
             &_motorRTdata.feedback_info[4],      //
             &_motorRTdata.feedback_info[5],      //
             &_motorRTdata.feedback_info[6],      //
             &_motorRTdata.feedback_info[7],      //
             &_motorRTdata.feedback_info[8],      //
             &_motorRTdata.feedback_info[9],      //
             &_motorRTdata.feedback_info[10],     //
             &_motorRTdata.feedback_info[11],     //
             &_motorRTdata.feedback_info[12],     //
             &_motorRTdata.feedback_info[13],     //
             &_motorRTdata.feedback_info[14],     //
             &_motorRTdata.feedback_info[15],     //
             &_motorRTdata.feedback_info[16],     //
             &_motorRTdata.feedback_info[17],     //
             &_motorRTdata.feedback_info[18],     //
             &_motorRTdata.feedback_info[19],     //
             &_motorRTdata.feedback_info[20],     //
             &_motorRTdata.feedback_info[21],     //
             &_motorRTdata.feedback_info[22],     //
             &_motorRTdata.feedback_info[23],     //
             &_motorRTdata.feedback_info[24],     //
             &_motorRTdata.feedback_info[25],     //
             &_motorRTdata.feedback_info[26],     //
             &_motorRTdata.feedback_info[27],     //
             &_motorRTdata.feedback_info[28],     //
             &_motorRTdata.feedback_info[29],     //
             &_motorRTdata.feedback_info[30],     //
             &_motorRTdata.feedback_info[31],     //
             &_motorRTdata.feedback_info[32],     //
             &_motorRTdata.feedback_info[33],     //
             &_motorRTdata.feedback_info[34],     //
             &_motorRTdata.feedback_info[35]      // motor-feedback_info
      );
      gui_connection_count = 0;
    } else {
      recv_buffer = "error";
      ++gui_connection_count;
    }
  }

  void senddata2vessel() {
    static int i = 0;
    ++i;
    send_buffer.clear();
    send_buffer = "$IPAC, 0.1, 0.2, 0.3,1111111" + std::to_string(i);

    send_buffer += "\n";

    size_t bytes_wrote = my_serial.write(send_buffer);
  }
};

#endif /* _GUICLIENT_H_ */