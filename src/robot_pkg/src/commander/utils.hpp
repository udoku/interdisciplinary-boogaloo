#ifndef __COMMANDER_UTILS__
#define __COMMANDER_UTILS__

#include "common/common_header.hpp"

namespace Timeout {
/** Run a function with a timeout. Returns the return value of the function, unless it times out in
 * which case returns false.
 * Time out is in seconds. If it is negative, no timeout is enabled. Task name is used for printing
 * debug info. */
bool runWithTimeout(function<bool()> code, int timeout, string name = string("<anonymous task>"));
} // namespace Timeout

/** Basic functionality to get the robot to move places */
namespace Motion {

/** Initializes publishers/subscribers for motion namespace */
void init(ros::NodeHandle nh);

/** Handle incoming robot states */
void handleNewRobotStateMsg(robot_pkg::RobotState msg);

/** Check if mobility has recieved a robot state */
bool hasCurrentState();

robot_pkg::RobotState getCurrentState();

} // namespace Motion

/** Advanced mobility code (for example, search patterns) */
namespace Recipes {

} // namespace Recipes

/** Functionality to communicate with vision */
namespace Vision {
void init(ros::NodeHandle nh);

// void handleNewVisionData(const crt_system::Detections::ConstPtr& detections);

// /** Tell the vision process to use the given detector. */
// void setDetector(DetectorType detector_type, int mode = 0);

// // Clears the current cached vision data.
// void clearVisionData();

// /** Get the most recent vision response for the object 'object_type'*/
// vector<Detection> getCurrentVisionData(DetectionType object_type);

// /** Wait for some data for DetectionType to show up. (returns false if it
// times out first (timeout is in seconds). Stores the data in result if it is non-null */
// bool waitVisionData(DetectionType object_type, double timeout, vector<Detection>* result = NULL,
//                       unsigned int min_num_detections = 1, unsigned int max_num_detections = 1e6);
// bool waitVisionData(vector<DetectionType> object_types, double timeout,
//                       vector<Detection>* result = NULL, unsigned int min_num_detections = 1,
//                       unsigned int max_num_detections = 1e6);
} // namespace Vision

/** Functionality to manipulate non-mobility actuators */
namespace Actuators {

/** Initializes publishers/subscribers for actuators namespace */
void init(ros::NodeHandle nh);

} // namespace Actuators

namespace System {
/** Initializes publishers/subscribers for system namespace */
void init(ros::NodeHandle nh);

/* Command a full system-wide reset */
void systemWideReset();
} // namespace System

#endif /** __UTILS__ **/
