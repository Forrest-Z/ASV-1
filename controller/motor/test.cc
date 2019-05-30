#include "motorclient.h"

int main() {
  motorclient _motorclient;
  _motorclient.startup_socket_client();
  _motorclient.PLCcommunication();
  return 0;
}