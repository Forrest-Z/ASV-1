#ifndef _MOTORCLIENTDATA_H_
#define _MOTORCLIENTDATA_H_

union command_data {
  float a[12];
  char b[48];
};

union read_data {
  int a[60];
  char b[240];
};

// real-time data in the servo
template <int m>
struct motorRTdata {
  float command_alpha[m];
  float command_rotation[m];
  int feedback_alpha[m];
  int feedback_rotation[m];
  int feedback_torque[2 * m];
  int feedback_info[6 * m];  // run/warning/alarm
};

#endif /* _MOTORCLIENTDATA_H_ */