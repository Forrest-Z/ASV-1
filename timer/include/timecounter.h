/*
***********************************************************************
* timecounter.h: thread-based DP controller and network
* function to run the whole loop on server (including PN server,
* 6D motion capture, Kalman, PID, thruster allocation, joystick,
* save2sqlite, viewer).
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _TIMECOUNTER_H_
#define _TIMECOUNTER_H_
#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/timer.hpp>
#include <iomanip>

class timecounter {
  using T_BOOST_CLOCK =
      boost::date_time::microsec_clock<boost::posix_time::ptime>;

 public:
  timecounter() : t_start(T_BOOST_CLOCK::local_time()){};

  // return the elapsed duration in milliseconds
  long int timeelapsed() {
    boost::posix_time::ptime t_now(T_BOOST_CLOCK::local_time());
    boost::posix_time::time_duration t_elapsed = t_now - t_start;
    t_start = t_now;
    return t_elapsed.total_milliseconds();
  }
  ~timecounter() {}

 private:
  boost::posix_time::ptime t_start;
};

#endif /*_TIMECOUNTER_H_*/