#include <cstdio>
#include "gps.h"
using std::setprecision;
int main() {
  gpsRTdata gps_data{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  std::string buffer;
  try {
    gpsimu _gpsimu(51, true, 115200);  // zone 30n
    while (1) {
      gps_data = _gpsimu.gpsonestep().getgpsRTdata();
      buffer = _gpsimu.getserialbuffer();
      std::cout << "buffer=    " << buffer;
      std::cout << "date:      " << gps_data.date << std::endl;
      std::cout << "time:      " << std::fixed << setprecision(3)
                << gps_data.time << std::endl;
      std::cout << "heading:   " << std::fixed << setprecision(3)
                << gps_data.heading << std::endl;
      std::cout << "pitch:     " << std::fixed << setprecision(3)
                << gps_data.pitch << std::endl;
      std::cout << "roll:      " << std::fixed << setprecision(3)
                << gps_data.roll << std::endl;
      std::cout << "latitud:   " << std::fixed << setprecision(7)
                << gps_data.latitude << std::endl;
      std::cout << "longitude: " << std::fixed << setprecision(7)
                << gps_data.longitude << std::endl;
      std::cout << "UTM_x:     " << std::fixed << setprecision(7)
                << gps_data.UTM_x << std::endl;
      std::cout << "UTM_y:     " << std::fixed << setprecision(7)
                << gps_data.UTM_y << std::endl;
      std::cout << "altitude:  " << std::fixed << setprecision(2)
                << gps_data.altitude << std::endl;
      std::cout << "speed_v:   " << std::fixed << setprecision(3) << gps_data.Ve
                << std::endl;
      std::cout << "speed_u:   " << std::fixed << setprecision(3) << gps_data.Vn
                << std::endl;
      std::cout << "speed_n:   " << std::fixed << setprecision(3) << gps_data.Vu
                << std::endl;
      std::cout << "basinLine  " << std::fixed << setprecision(3)
                << gps_data.base_line << std::endl;
      std::cout << "NSV1:      " << gps_data.NSV1 << std::endl;
      std::cout << "NSV2:      " << gps_data.NSV2 << std::endl;
      switch (gps_data.status2) {
        case '0':
          std::cout << "Satus:     GPS初始化" << std::endl;
          break;
        case '1':
          std::cout << "Satus:     粗对准" << std::endl;
          break;
        case '2':
          std::cout << "Satus:     精对准" << std::endl;
          break;
        case '3':
          std::cout << "Satus:     GPS定位" << std::endl;
          break;
        case '4':
          std::cout << "Satus:     GPS定向" << std::endl;
          break;
        case '5':
          std::cout << "Satus:     GPS RTK" << std::endl;
          break;
        case 'B':
          std::cout << "Satus:     差分定向" << std::endl;
          break;
        default:
          std::cout << "Satus:     状态未知" << std::endl;
      }
      std::cout << std::endl;
      my_sleep(100);
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
}
