#include "motorclient.h"

void test1() {
  motorclient _motorclient;
  motorRTdata<6> testmotorRTdata;
  _motorclient.startup_socket_client();
  sleep(10);
  for (int i = 0; i != 100; ++i) {
    for (int j = 0; j != 6; ++j) {
      testmotorRTdata.command_alpha[j] = 3 * i;
      testmotorRTdata.command_rotation[j] = 100;
    }
    _motorclient.PLCcommunication(testmotorRTdata);
    printf("Positon :");
    for (int i = 0; i < 6; i++) {
      printf("%d  ", testmotorRTdata.feedback_alpha[i]);
    }
    printf("\n");

    printf("Velocity:");
    for (int i = 0; i < 6; i++) {
      printf("%d  ", testmotorRTdata.feedback_rotation[i]);
    }
    printf("\n");

    printf("Torque:");
    for (int i = 0; i < 12; i++) {
      printf("%d  ", testmotorRTdata.feedback_torque[i]);
    }
    printf("\n");
    usleep(100000);
  }
}

void test2() {
  motorclient _motorclient;
  motorRTdata<6> testmotorRTdata;
  _motorclient.startup_socket_client();

  _motorclient.TerminalPLC();
}
int main() {
  test2();
  return 0;
}