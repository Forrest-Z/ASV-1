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
#endif /* _MOTORCLIENTDATA_H_ */