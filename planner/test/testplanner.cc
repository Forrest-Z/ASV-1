/*
*******************************************************************************
* testplanner.cc:
* unit test for path following using line of sight algorithm
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "planner.h"
#include "utilityio.h"

INITIALIZE_EASYLOGGINGPP

int main() {
  el::Loggers::addFlag(el::LoggingFlag::CreateLoggerAutomatically);

  plannerdata _plannerdata{
      0.1,  // sample_time
      1,    // los_radius
      0     // los_capture_radius
  };
  plannerRTdata _plannerRTdata{
      Eigen::Vector3d::Zero(),  // setpoint
      Eigen::Vector3d::Zero(),  // v_setpoint
      Eigen::Vector2d::Zero(),  // waypoint0
      Eigen::Vector2d::Zero(),  // waypoint1
      Eigen::Vector3d::Zero()   // command
  };
  _plannerRTdata.v_setpoint(0) = 0.1;
  Eigen::Vector2d startposition = (Eigen::Vector2d() << 2, 0.1).finished();
  Eigen::Vector2d endposition = Eigen::Vector2d::Zero();
  double radius = 3;
  planner _planner(_plannerdata);

  auto waypoints = _planner.followcircle(startposition, endposition, radius,
                                         0.3, _plannerRTdata.v_setpoint(0));

  waypoints = waypoints.transpose().eval();
  utilityio _utilityio;
  _utilityio.write2csvfile("csvfile.csv", waypoints);
}