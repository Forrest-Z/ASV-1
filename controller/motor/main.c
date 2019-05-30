#include "client.h"

int main() {
  char IP_Address[15] = SERVER_IP;
  startup_socket_client(IP_Address);
  return 0;
}