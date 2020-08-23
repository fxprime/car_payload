#pragma once
/* Copyright (C) Thanabdee Bulunseechart, Inc - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Thanabdee Bulunseechart <paheyisoicus@gmail.com>, August 2018
 */

// Standard includes
#include <iostream>
#include <vector>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif


using std::string;
using namespace std;




class serial
{
public:

  /**


     Returns the file descriptor on success or -1 on error.
  */

  int open_port(std::string& port);
  string port() const { return _port;};

  bool isOpened();

  void close_port();

  serial(std::string& device_name);
  ~serial();


  int Write(const char* data, uint32_t size);
  int Read(char* data, uint32_t size);




  bool setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);





  // Checks if a byte is available in the serial port
  int BytesAvailable();


  bool waitForReadyRead(int ms);
private:
  int fd_;
  string _port;
};


