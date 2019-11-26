#ifndef __COMMON_H__
#define __COMMON_H__

#include <eigen3/Eigen/Geometry>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <ros/ros.h>
#include "constants.hpp"

// Msgs
#include <robot_pkg/RobotState.h>
#include <robot_pkg/MotionTarget.h>
#include <robot_pkg/ServoCommand.h>
#include <robot_pkg/Detector.h>
#include <robot_pkg/Detection.h>
#include <robot_pkg/Detections.h>
#include <robot_pkg/Frame.h>
#include <robot_pkg/UltrasonicPing.h>
#include <robot_pkg/LedCommand.h>
#include <std_msgs/Bool.h>

// Srvs
#include <robot_pkg/RunDetector.h>
#include <robot_pkg/SetupDetector.h>

using namespace Eigen;
using namespace std;

long long getCurrentMsCount();


#endif