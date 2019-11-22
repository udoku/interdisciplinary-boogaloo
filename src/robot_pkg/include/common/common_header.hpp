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
#include <std_msgs/Bool.h>

using namespace Eigen;
using namespace std;

long long getCurrentMsCount();


#endif