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

class guiclient {
 public:
  guiclient()
      : my_serial("/dev/ttyS4", 19200, serial::Timeout::simpleTimeout(500)),
        send_buffer(""),
        recv_buffer("") {
    enumerate_ports();
    judgeserialstatus();
  }
  ~guiclient() {}

  void guicommunication() {
    timecounter _timer;
    senddata2gui();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(500 - _timer.timeelapsed()));
    parsedatafromgui();
    std::cout << "send message:" << send_buffer << std::endl;
    std::cout << "recv message:" << recv_buffer << std::endl;
  }

 private:
  serial::Serial my_serial;
  std::string send_buffer;
  std::string recv_buffer;

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

  void parsedatafromgui() {
    recv_buffer = my_serial.readline(400, "\n");
    double test_A = 0.0;
    double test_B = 0.0;
    double test_C = 0.0;
    std::size_t pos = recv_buffer.find("$IPAC");
    if (pos != std::string::npos) {
      recv_buffer = recv_buffer.substr(pos);
      sscanf(recv_buffer.c_str(), "$IPAC,%lf,%lf,%lf",
             &test_A,  // date
             &test_B,  // time
             &test_C   // heading
      );

    } else
      recv_buffer = "error";
  }

  void senddata2gui() {
    static int i = 0;
    ++i;
    send_buffer.clear();
    send_buffer = "$IPAC, 0.1, 0.2, 0.3,1111111" + std::to_string(i);

    send_buffer += "\n";

    size_t bytes_wrote = my_serial.write(send_buffer);
  }
};

#endif /* _GUICLIENT_H_ */