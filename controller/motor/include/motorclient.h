#ifndef _MOTORCLIENT_H
#define _MOTORCLIENT_H

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>  //delay function
#include "motorclientdata.h"

class motorclient {
 public:
  motorclient() {}
  ~motorclient() { close(sockfd); }

  void commandfromcontroller(float *_command_alpha, float *_command_rotation,
                             const Eigen::Matrix<int, 6, 1> &_alpha_deg,
                             const Eigen::Matrix<int, 6, 1> &_rotation) {
    for (int i = 0; i != 6; ++i) {
      _command_alpha[i] = static_cast<float>(_alpha_deg(i));
      _command_rotation[i] = static_cast<float>(_rotation(i));
    }
  }

  void PLCcommunication(motorRTdata<6> &_motorRTdata) {
    commandPLC(_motorRTdata.command_alpha, _motorRTdata.command_rotation);
    readPLC(_motorRTdata);
  }

  void TerminalPLC() {
    // printf("%x\n",read_buf[0]);
    printf("sockfd = %d\n", sockfd);
    motorRTdata<6> _motorRTdata;
    while (1) {
      printf(
          "reset(1) / run(2) / command(3) / read(4) / stop(5) / suspend(6) "
          "?\n");
      char flag;
      scanf("%c", &flag);
      getchar();

      switch (flag) {
        case '1':  // reset=1 to 0
          resetPLC();
          break;
        case '2':  // run=1 to 0
          runPLC();
          break;
        case '3':  // command
          commandPLC();
          break;
        case '4':  // read
          readPLC(_motorRTdata);
          printf("Positon :");
          for (int i = 0; i < 6; i++) {
            printf("%d  ", _motorRTdata.feedback_alpha[i]);
          }
          printf("\n");

          printf("Volecity:");
          for (int i = 0; i < 6; i++) {
            printf("%d  ", _motorRTdata.feedback_rotation[i]);
          }
          printf("\n");

          printf("Torque:");
          for (int i = 0; i < 12; i++) {
            printf("%d  ", _motorRTdata.feedback_torque[i]);
          }
          printf("\n");
          break;
        case '5':  // stop=1 to 0
          stopPLC();
          break;
        case '6':  // suspend
          break;
      }
    }
    //******************************  PLC  ******************************
  }

  void startup_socket_client() {
    struct addrinfo hints, *servinfo, *p;
    int rv;
    char s[INET6_ADDRSTRLEN];

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if ((rv = getaddrinfo("192.168.1.1", "10001", &hints, &servinfo)) != 0) {
      fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    }

    // 用循环取得全部的结果，并先连接到能成功连接的
    for (p = servinfo; p != NULL; p = p->ai_next) {
      if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) ==
          -1) {
        perror("client: socket");
        continue;
      }

      if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
        close(sockfd);
        perror("client: connect");
        continue;
      }
      break;
    }

    if (p == NULL) {
      fprintf(stderr, "client: failed to connect\n");
    }

    inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr), s,
              sizeof s);
    printf("client: connecting to %s\n", s);
    freeaddrinfo(servinfo);  // 全部皆以这个 structure 完成

    //******************************  PLC  ******************************

    mk_read_command_data(read_buf);
    mk_command_command_data(command_buf);
    mk_reset_command_data(reset_buf);
    mk_run_command_data(run_buf);
    mk_stop_clean_command_data(stop_clean_buf);

    resetPLC();
    sleep(40);
    runPLC();
  }

 private:
  int sockfd;
  union command_data _command_data;
  union read_data _read_data;
  char read_buf[128];
  char rbuf[1024];
  char command_buf[128];
  char reset_buf[128];
  char run_buf[128];
  char stop_clean_buf[128];

  void union_transmit(char *_command_buf) {
    for (int i = 0; i < 48; i++) {
      _command_buf[i + 22] = _command_data.b[i];
    }
  }

  void min_position(void) {
    float delta_position[6];
    static float last_position[6];
    for (int i = 0; i < 6; ++i) {
      delta_position[i] = _command_data.a[i * 2] - last_position[i];
      if (delta_position[i] > 180) {
        delta_position[i] = delta_position[i] - 360;
      }
      if (delta_position[i] < -180) {
        delta_position[i] = delta_position[i] + 360;
      }
      _command_data.a[i * 2] = last_position[i] + delta_position[i];
      last_position[i] = _command_data.a[i * 2];
    }
  }
  // 读取扩展 MEMOBUS 协议、保持寄存器的内容 (SFC=09)，制作命令
  void mk_read_command_data(char *_read_buf) {
    // 218 标题部的制作
    // 数据类型设定
    _read_buf[0] = 0x11;  // 扩展 MEMOBUS (指令命令)
    _read_buf[1] = 0x00;  // 串行编号设定 (每次发送时增加)
    _read_buf[2] =
        0x01;  // 设定发送目标的通道编号 PLC 侧通道不固定，因此可固定为 0
    _read_buf[3] =
        0x00;  // 设定发送目标的通道编号 因电脑中无通道概念，固定为 0。
    _read_buf[4] = 0x00;  // 备用
    _read_buf[5] = 0x00;  // 备用
    _read_buf[6] =
        0x16;  // 所有数据数设定 (从 218 标题的首部到 MEMOBUS 数据的最后) // L
               // (22 字节 =218 标题 (12 字节)+ MEMOBUS 数据 (10 字节))
    _read_buf[7] = 0x00;   // H
    _read_buf[8] = 0x00;   // 备用
    _read_buf[9] = 0x00;   // 备用
    _read_buf[10] = 0x00;  // 备用
    _read_buf[11] = 0x00;  // 备用

    // MEMOBUS 数据部的制作
    _read_buf[12] =
        0x08;  // MEMOBUS 数据长度 (L) Length 为从 MFC 开始到数据的最后
    _read_buf[13] = 0x00;  // MEMOBUS 数据长度 (H)
    _read_buf[14] = 0x20;  // MFC 固定为 0x20
    _read_buf[15] = 0x09;  // SFC 为 0x09 (读取保持寄存器的内容 (扩展))
    _read_buf[16] = 0x10;  // CPU 编号设定
    _read_buf[17] = 0x00;
    _read_buf[18] = (char)0xA0;  // start address(WL04000 )
    _read_buf[19] = 0x0F;
    _read_buf[20] = 0x79;  // 寄存器数设定(from WL04000 to WL04118, total
                           // numuber: 240byte(0xF0)
    _read_buf[21] = 0x00;  // 还要除以2！！！，因为寄存器是两个字节(0x78))
  }

  void mk_reset_command_data(char *_reset_buf) {
    // 218 标题部的制作
    // 数据类型设定
    _reset_buf[0] = 0x11;  // 扩展 MEMOBUS (指令命令)
    _reset_buf[1] = 0x00;  // 串行编号设定 (每次发送时增加)
    _reset_buf[2] =
        0x01;  // 设定发送目标的通道编号 PLC 侧通道不固定，因此可固定为 0
    _reset_buf[3] =
        0x00;  // 设定发送目标的通道编号 因电脑中无通道概念，固定为 0。
    _reset_buf[4] = 0x00;  // 备用
    _reset_buf[5] = 0x00;  // 备用
    // 所有数据数设定 (从 218 标题的首部到 MEMOBUS 数据的最后)
    _reset_buf[6] =
        0x18;  // L (24 字节 =218 标题 (12 字节)+ MEMOBUS 数据 (10 字节)+2)
    _reset_buf[7] = 0x00;   // H
    _reset_buf[8] = 0x00;   // 备用
    _reset_buf[9] = 0x00;   // 备用
    _reset_buf[10] = 0x00;  // 备用
    _reset_buf[11] = 0x00;  // 备用

    // MEMOBUS 数据部的制作
    // Length 为从 MFC 开始到数据的最后
    _reset_buf[12] = 0x0A;  // MEMOBUS 数据长度 (L)8+2=10
    _reset_buf[13] = 0x00;  // MEMOBUS 数据长度 (H)
    _reset_buf[14] = 0x20;  // MFC 固定为 0x20
    _reset_buf[15] = 0x0B;  // SFC 为 0x09 (读取保持寄存器的内容 (扩展))
    _reset_buf[16] = 0x10;  // CPU 编号设定
    _reset_buf[17] = 0x00;
    _reset_buf[18] = (char)0xD0;  // start address (MF03024-0x0BD0)
    _reset_buf[19] = 0x0B;
    _reset_buf[20] = 0x01;  // 寄存器数设定
    _reset_buf[21] = 0x00;
    // data for writing
    _reset_buf[22] = 0x00;
    _reset_buf[23] = 0x00;
  }

  void mk_run_command_data(char *_run_buf) {
    // 218 标题部的制作
    // 数据类型设定
    _run_buf[0] = 0x11;  // 扩展 MEMOBUS (指令命令)
    _run_buf[1] = 0x00;  // 串行编号设定 (每次发送时增加)
    _run_buf[2] =
        0x01;  // 设定发送目标的通道编号 PLC 侧通道不固定，因此可固定为 0
    _run_buf[3] =
        0x00;  // 设定发送目标的通道编号 因电脑中无通道概念，固定为 0。
    _run_buf[4] = 0x00;  // 备用
    _run_buf[5] = 0x00;  // 备用
    // 所有数据数设定 (从 218 标题的首部到 MEMOBUS 数据的最后)
    _run_buf[6] =
        0x18;  // L (24 字节 =218 标题 (12 字节)+ MEMOBUS 数据 (10 字节)+2)
    _run_buf[7] = 0x00;   // H
    _run_buf[8] = 0x00;   // 备用
    _run_buf[9] = 0x00;   // 备用
    _run_buf[10] = 0x00;  // 备用
    _run_buf[11] = 0x00;  // 备用

    // MEMOBUS 数据部的制作
    // Length 为从 MFC 开始到数据的最后
    _run_buf[12] = 0x0A;  // MEMOBUS 数据长度 (L)8+2=10
    _run_buf[13] = 0x00;  // MEMOBUS 数据长度 (H)
    _run_buf[14] = 0x20;  // MFC 固定为 0x20
    _run_buf[15] = 0x0B;  // SFC 为 0x09 (读取保持寄存器的内容 (扩展))
    _run_buf[16] = 0x10;  // CPU 编号设定
    _run_buf[17] = 0x00;
    _run_buf[18] = (char)0xD1;  // start address (MF03025-0x0BD1)
    _run_buf[19] = 0x0B;
    _run_buf[20] = 0x01;  // 寄存器数设定
    _run_buf[21] = 0x00;
    // data for writing
    _run_buf[22] = 0x00;
    _run_buf[23] = 0x00;
  }

  void mk_stop_clean_command_data(char *_stop_clean_buf) {
    // 218 标题部的制作
    // 数据类型设定
    _stop_clean_buf[0] = 0x11;  // 扩展 MEMOBUS (指令命令)
    _stop_clean_buf[1] = 0x00;  // 串行编号设定 (每次发送时增加)
    _stop_clean_buf[2] =
        0x01;  // 设定发送目标的通道编号 PLC 侧通道不固定，因此可固定为 0
    _stop_clean_buf[3] =
        0x00;  // 设定发送目标的通道编号 因电脑中无通道概念，固定为 0。
    _stop_clean_buf[4] = 0x00;  // 备用
    _stop_clean_buf[5] = 0x00;  // 备用
    // 所有数据数设定 (从 218 标题的首部到 MEMOBUS 数据的最后)
    _stop_clean_buf[6] =
        0x2E;  // L (46 字节 =218 标题 (12 字节)+ MEMOBUS 数据 (10 字节)+24)
    _stop_clean_buf[7] = 0x00;   // H
    _stop_clean_buf[8] = 0x00;   // 备用
    _stop_clean_buf[9] = 0x00;   // 备用
    _stop_clean_buf[10] = 0x00;  // 备用
    _stop_clean_buf[11] = 0x00;  // 备用

    // MEMOBUS 数据部的制作
    // Length 为从 MFC 开始到数据的最后
    _stop_clean_buf[12] = 0x20;  // MEMOBUS 数据长度 (L)8+24=32
    _stop_clean_buf[13] = 0x00;  // MEMOBUS 数据长度 (H)
    _stop_clean_buf[14] = 0x20;  // MFC 固定为 0x20
    _stop_clean_buf[15] = 0x0B;  // SFC 为 0x09 (读取保持寄存器的内容 (扩展))
    _stop_clean_buf[16] = 0x10;  // CPU 编号设定
    _stop_clean_buf[17] = 0x00;
    _stop_clean_buf[18] = (char)0xD2;  // start address (MW03026-0x0BD2)
    _stop_clean_buf[19] = 0x0B;
    _stop_clean_buf[20] = 0x0C;  // 寄存器数设定
    _stop_clean_buf[21] = 0x00;
    // data for writing
    // stop_clean_buf[22] = 0x01;
    // stop_clean_buf[23] = 0x00;
  }

  void mk_command_command_data(char *_command_buf) {
    // 218 标题部的制作
    // 数据类型设定
    _command_buf[0] = 0x11;  // 扩展 MEMOBUS (指令命令)
    _command_buf[1] = 0x00;  // 串行编号设定 (每次发送时增加)
    _command_buf[2] =
        0x01;  // 设定发送目标的通道编号 PLC 侧通道不固定，因此可固定为 0
    _command_buf[3] =
        0x00;  // 设定发送目标的通道编号 因电脑中无通道概念，固定为 0。
    _command_buf[4] = 0x00;  // 备用
    _command_buf[5] = 0x00;  // 备用
    // 所有数据数设定 (从 218 标题的首部到 MEMOBUS 数据的最后)
    _command_buf[6] =
        0x46;  // L (70 字节 =218 标题 (12 字节)+ MEMOBUS 数据 (10 字节)+48)
    _command_buf[7] = 0x00;   // H
    _command_buf[8] = 0x00;   // 备用
    _command_buf[9] = 0x00;   // 备用
    _command_buf[10] = 0x00;  // 备用
    _command_buf[11] = 0x00;  // 备用

    // MEMOBUS 数据部的制作
    // Length 为从 MFC 开始到数据的最后
    _command_buf[12] = 0x38;  // MEMOBUS 数据长度 (L)>>8+48=56(0x38)
    _command_buf[13] = 0x00;  // MEMOBUS 数据长度 (H)
    _command_buf[14] = 0x20;  // MFC 固定为 0x20
    _command_buf[15] = 0x0B;  // SFC 为 0x09 (读取保持寄存器的内容 (扩展))
    _command_buf[16] = 0x10;  // CPU 编号设定
    _command_buf[17] = 0x00;
    _command_buf[18] = (char)0xB8;  // start address (MF03000-0x0BB8)
    _command_buf[19] = 0x0B;
    _command_buf[20] = 0x18;  // 寄存器数设定(24)
    _command_buf[21] = 0x00;
    // data for writing
    // 1P
    _command_buf[22] = 0x00;
    _command_buf[23] = 0x00;
    _command_buf[24] = 0x00;
    _command_buf[25] = 0x00;
    // 1V
    _command_buf[26] = 0x00;
    _command_buf[27] = 0x00;
    _command_buf[28] = 0x00;
    _command_buf[29] = 0x00;
    // 2P
    _command_buf[30] = 0x00;
    _command_buf[31] = 0x00;
    _command_buf[32] = 0x00;
    _command_buf[33] = 0x00;
    // 2V
    _command_buf[34] = 0x00;
    _command_buf[35] = 0x00;
    _command_buf[36] = 0x00;
    _command_buf[37] = 0x00;
    // 3P
    _command_buf[38] = 0x00;
    _command_buf[39] = 0x00;
    _command_buf[40] = 0x00;
    _command_buf[41] = 0x00;
    // 3V
    _command_buf[42] = 0x00;
    _command_buf[43] = 0x00;
    _command_buf[44] = 0x00;
    _command_buf[45] = 0x00;
    // 4P
    _command_buf[46] = 0x00;
    _command_buf[47] = 0x00;
    _command_buf[48] = 0x00;
    _command_buf[49] = 0x00;
    // 4V
    _command_buf[50] = 0x00;
    _command_buf[51] = 0x00;
    _command_buf[52] = 0x00;
    _command_buf[53] = 0x00;
    // 5P
    _command_buf[54] = 0x00;
    _command_buf[55] = 0x00;
    _command_buf[56] = 0x00;
    _command_buf[57] = 0x00;
    // 5V
    _command_buf[58] = 0x00;
    _command_buf[59] = 0x00;
    _command_buf[60] = 0x00;
    _command_buf[61] = 0x00;
    // 6P
    _command_buf[62] = 0x00;
    _command_buf[63] = 0x00;
    _command_buf[64] = 0x00;
    _command_buf[65] = 0x00;
    // 6V
    _command_buf[66] = 0x00;
    _command_buf[67] = 0x00;
    _command_buf[68] = 0x00;
    _command_buf[69] = 0x00;
  }
  // 响应数据的检查
  int chk_rsp_command_data(motorRTdata<6> &_motorRTdata, int rlen) {
    // 所有数据长度的检查
    if (rlen != 262)
      return (-1);  // 读取10字对应的响应为40字节,
                    // (218标题(12字节)+MEMOBUS数据(28字节))

    // 数据包类型检查
    if (rbuf[0] != 0x19) return (-2);  // 非 MEMOBUS 响应

    // 串行编号检查
    if (read_buf[1] != rbuf[1]) return (-3);  // 与命令的串行编号不一致

    // 传送文件中的所有数据长度的检查
    if ((rbuf[6] != 0x04) && (rbuf[7] != 0x01))
      return (-4);  // 260 字节 = 218 标题 (12 字节 )+MEMOBUS 数据 (8 + 240 =
                    // 248字节 )

    if (rbuf[12] != -8) {
      printf("111\n");
      printf("%d\n", rbuf[12]);
    }
    if (rbuf[13] != 0x00) printf("222\n");

    // MEMOBUS 数据长度检查
    // if ((rbuf[12] != 0xF6) || (rbuf[13] != 0x00)) return (-5);  // 6+240=246
    // 字节（看手册说明为什么是“6”） MFC 的检查
    if (rbuf[14] != 0x20) return (-6);  // MFC 固定为 0x20

    // SFC 的检查
    if (rbuf[15] != 0x09) return (-7);  // SFC 为 0x09 (读取保持寄存器的内容)

    // 寄存器数的检查
    if ((rbuf[18] != 0x79) || (rbuf[19] != 0x00)) return (-8);  // 非 240 bytes

    // 读取寄存器数据 rbuf[20] 以后
    for (int i = 0; i < 240; i++) {
      _read_data.b[i] = rbuf[i + 20];
    }

    for (int i = 0; i < 6; i++) {
      _motorRTdata.feedback_alpha[i] =
          static_cast<int>(_read_data.a[i] / 1000.0);
      _motorRTdata.feedback_rotation[i] =
          static_cast<int>(_read_data.a[i + 6] / 6000.0);
    }

    for (int i = 0; i < 12; i++)
      _motorRTdata.feedback_torque[i] = _read_data.a[i + 12];
    for (int i = 0; i < 36; i++)
      _motorRTdata.feedback_info[i] = _read_data.a[i + 24];

    return 0;
  }

  // 取得 IPv4 或 IPv6 的 sockaddr：
  void *get_in_addr(struct sockaddr *sa) {
    if (sa->sa_family == AF_INET) {
      return &(((struct sockaddr_in *)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
  }

  void resetPLC() {
    reset_buf[22] = 0x01;
    int slen = send(sockfd, reset_buf, 24, 0);  //发送命令(70字节)
    if (slen != 24)  // 如果发送成功，则返回发送的字节数 (70字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }
    printf("before recv\n");
    int rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }

    usleep(200000);

    reset_buf[22] = 0x00;
    slen = send(sockfd, reset_buf, 24, 0);  //发送命令(70字节)
    if (slen != 24)  // 如果发送成功，则返回发送的字节数 (70字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }
    printf("before recv\n");
    rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }

    usleep(200000);

    // set 0 value for position and speed
    for (int i = 0; i < 48; i++) {
      command_buf[i + 22] = 0x00;
    }
    slen = send(sockfd, command_buf, 70, 0);  //发送命令(70字节)
    if (slen != 70)  // 如果发送成功，则返回发送的字节数 (70字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }
    printf("before recv\n");
    rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }
  }

  void runPLC() {
    run_buf[22] = 0x01;
    int slen = send(sockfd, run_buf, 24, 0);  //发送命令(70字节)
    if (slen != 24)  // 如果发送成功，则返回发送的字节数 (70字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }
    printf("before recv\n");
    int rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }

    usleep(200000);

    run_buf[22] = 0x00;
    slen = send(sockfd, run_buf, 24, 0);  //发送命令(70字节)
    if (slen != 24)  // 如果发送成功，则返回发送的字节数 (70字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }
    printf("before recv\n");
    rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }
  }

  void commandPLC(float *alpha, float *rotation) {
    for (int i = 0; i != 6; ++i) _command_data.a[2 * i] = alpha[i];
    min_position();
    for (int i = 0; i != 6; ++i) _command_data.a[2 * i + 1] = rotation[i];
    union_transmit(command_buf);

    int slen = send(sockfd, command_buf, 70, 0);  //发送命令(70字节)
    if (slen != 70)  // 如果发送成功，则返回发送的字节数 (70字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }
    printf("before recv\n");
    int rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }
  }

  void commandPLC() {
    printf("Positon:");
    scanf("%f %f %f %f %f %f", &_command_data.a[0], &_command_data.a[2],
          &_command_data.a[4], &_command_data.a[6], &_command_data.a[8],
          &_command_data.a[10]);
    getchar();  // get return

    min_position();

    printf("Volecity:");
    scanf("%f %f %f %f %f %f", &_command_data.a[1], &_command_data.a[3],
          &_command_data.a[5], &_command_data.a[7], &_command_data.a[9],
          &_command_data.a[11]);
    getchar();  // get return

    union_transmit(command_buf);

    int slen = send(sockfd, command_buf, 70, 0);  //发送命令(70字节)
    if (slen != 70)  // 如果发送成功，则返回发送的字节数 (70字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }
    printf("before recv\n");
    int rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }
  }

  void readPLC(motorRTdata<6> &_motorRTdata) {
    // 发送命令数据
    // 主控制器不能发送数据时，该处理不会结束。
    int slen = send(sockfd, read_buf, 22, 0);  //发送命令(22字节)

    if (slen != 22)  // 如果发送成功，则返回发送的字节数 (22 字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }

    // 接收响应数据
    // 子控制器没有发送数据时，该处理不会结束。
    printf("before recv\n");
    int rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }

    // 响应数据的检查
    int rc = chk_rsp_command_data(_motorRTdata, rlen);
    printf("rc=%d\n", rc);
    if (rc != 0)  //接收数据异常
    {
      close(sockfd);
    }
    read_buf[1]++;  // 增加 218 标题的串行编号
  }

  void stopPLC() {
    for (int i = 0; i < 24; i++) {
      stop_clean_buf[i + 22] = (char)0xFF;
    }
    int slen = send(sockfd, stop_clean_buf, 46, 0);  //发送命令(70字节)
    if (slen != 46)  // 如果发送成功，则返回发送的字节数 (70字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }
    printf("before recv\n");
    int rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }

    usleep(200000);

    for (int i = 0; i < 24; i++) {
      stop_clean_buf[i + 22] = 0x00;
    }
    slen = send(sockfd, stop_clean_buf, 46, 0);  //发送命令(70字节)
    if (slen != 46)  // 如果发送成功，则返回发送的字节数 (70字节)。
    {
      close(sockfd);
      printf("Error: Send !! -> %d\n", slen);
    }
    printf("before recv\n");
    rlen = recv(sockfd, rbuf, sizeof(rbuf), 0);  // 接收发自对方的响应数据
    printf("rlen=%d\n", rlen);
    if (rlen <= 0)  // 如果接收错误则返回 0 以下
    {
      close(sockfd);
      printf("Error: Recv !! -> %d\n", rlen);
      perror("recv");
    }
  }


};
#endif /* _MOTORCLIENTDATA_H_ */