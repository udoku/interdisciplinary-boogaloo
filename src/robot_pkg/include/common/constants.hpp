#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include "common_header.hpp"

/** PROCESS NAMES */
const char* const COMMAND_PROCESS = "command_process";
const char* const MOBILITY_PROCESS = "mobility_process";
const char* const HARDWARE_PROCESS = "hardware_process";
const char* const VISION_PROCESS = "vision_process";

/** TOPIC NAMES */
const char* const KILLSWITCH_TOPIC = "killswitch";
const char* const MOTION_TARGET_TOPIC = "motion_target";
const char* const ROBOT_STATE_TOPIC = "robot_state";
const char* const SERVO_COMMAND_TOPIC = "servo_command";
const char* const HARDWARE_RESET_TOPIC = "hardware_reset";

/** UPDATE RATES */
const double ROBOT_STATE_UPDATE_RATE_HZ = 10;

extern const int DUMMY_CONST;

#endif