#include "motorclient.h"

void test1() {
  motorclient _motorclient;
  motorRTdata<6> testmotorRTdata;
  _motorclient.startup_socket_client(testmotorRTdata);
  for (int i = 0; i != 120; ++i) {
    for (int j = 0; j != 6; ++j) {
      testmotorRTdata.command_alpha[j] = 3 * i;
      testmotorRTdata.command_rotation[j] = 1000 - 10 * i;
    }
    _motorclient.PLCcommunication(testmotorRTdata);

    printf("Positon :");
    for (int j = 0; j < 6; j++) {
      printf("%d  ", testmotorRTdata.feedback_alpha[j]);
    }
    printf("\n");

    printf("Velocity:");
    for (int j = 0; j < 6; j++) {
      printf("%d  ", testmotorRTdata.feedback_rotation[j]);
    }
    printf("\n");

    printf("Torque:");
    for (int j = 0; j < 12; j++) {
      printf("%d  ", testmotorRTdata.feedback_torque[j]);
    }
    printf("\n");
    printf("run/warning/alarm:");
    for (int j = 0; j < 36; j++) {
      printf("%d  ", testmotorRTdata.feedback_info[j]);
    }
    printf("\n");
    printf("All info of servos:");
    printf("%d \n", testmotorRTdata.feedback_allinfo);
    usleep(1000000);
  }
}

void test2() {
  motorclient _motorclient;
  motorRTdata<6> testmotorRTdata;
  _motorclient.startup_socket_client(testmotorRTdata);

  _motorclient.TerminalPLC();
}
int main() {
  test1();
  return 0;
}