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
const char* const TIC_TAC_DROP_TOPIC = "drop_tic_tac";
const char* const HARDWARE_RESET_TOPIC = "hardware_reset";
const char* const DESIRED_DETECTOR_TOPIC = "desired_detector";
const char* const CAMERA_IMAGES_TOPIC = "images/camera";
const char* const DETECTION_IMAGE_TOPIC = "images/detected";
const char* const DETECTIONS_TOPIC = "detections";
const char* const DETECTOR_SETUP_TOPIC = "detector_setup";
const char* const DETECTOR_RUN_TOPIC = "detector_run";

/** UPDATE RATES */
const double MOBILITY_UPDATE_RATE_HZ = 100;

/** MOBILITY PARAMETERS */
const double MAX_POS_ERROR = 0.01; // meters
const double MAX_ANG_ERROR = 0.01; // radians

const double MAX_SPEED = 0.1; // meters/s
const double MAX_TURN_SPEED = 0.1; // radians/s

const double SPEED_TO_POWER = 0.5; // multiply by speed to get servo power
const double TURN_TO_SPEED = 0.1; // multiply by rad/s to get necessary m/s of wheels

const double TRANSITION_WAIT_TIME = 1.0; // seconds

const int WHEEL_IDS[3] = {0, 1, 2};
const int PIVOT_IDS[3] = {3, 4, 5};

const double TURNING_SETPOINTS[3] = {0.0, 0.0, 0.0};
const double STRAIGHT_SETPOINTS[3] = {0.0, 0.0, 0.0};
const double STRAFE_SETPOINTS[3] = {0.0, 0.0, 0.0};

const double STRAIGHT_DIRECTIONS[3] = {1, 1, 1};
const double TURN_DIRECTIONS[3] = {1, 1, 1};

extern const int DUMMY_CONST;

#endif
