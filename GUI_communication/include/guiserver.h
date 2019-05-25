/*
***********************************************************************
* guiserver.h: communication for gui server
* This header file can be read by C++ compilers
*
*  by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _GUISERVER_H_
#define _GUISERVER_H_

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "serial/serial.h"
#include "timecounter.h"

class guiserver {
 public:
  guiserver()
      : my_serial("/dev/ttyUSB0", 19200, serial::Timeout::simpleTimeout(1000)) {
    if (my_serial.isOpen())
      std::cout << " serial initialization successful!" << std::endl;
  }
  ~guiserver() {}

  void testcommunication() {
    timecounter _timer;
    long int et = _timer.timeelapsed();
    my_serial.setTimeout(serial::Timeout::max(), 100, 0, 100, 0);

    std::string recv_buffer = "";
    int i = 0;
    while (1) {
      ++i;
      std::cout << "Iteration " << i << std::endl;

      std::string send_buffer = "send " + std::to_string(i + 1) + "\n";
      // star to time
      size_t bytes_wrote = my_serial.write(send_buffer);
      if (my_serial.waitReadable()) recv_buffer = my_serial.readline(20);
      my_serial.flush();
      et = _timer.timeelapsed();
      std::cout << "The string sent is: " << send_buffer
                << " bytes_wrote: " << bytes_wrote << std::endl;
      std::cout << "The string recv is: " << recv_buffer << std::endl;
      std::cout << "elapsed time:" << et << std::endl;

      // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

 private:
  serial::Serial my_serial;

  void enumerate_ports() {
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end()) {
      serial::PortInfo device = *iter++;
      std::cout << device.port.c_str() << ", " << device.description.c_str()
                << ", " << device.hardware_id.c_str() << std::endl;
    }
  }
};

#endif /* _GUISERVER_H_ */