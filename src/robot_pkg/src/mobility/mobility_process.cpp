#include "mobility_process.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, string(VISION_PROCESS));

    MobilityProcess* p = new MobilityProcess();
    p->run();
}

MobilityProcess::MobilityProcess() {
    killswitch_sub_ = nh_.subscribe(KILLSWITCH_TOPIC, 10, &MobilityProcess::processKillswitch, this);
    motion_target_sub_ = nh_.subscribe(MOTION_TARGET_TOPIC, 1, &MobilityProcess::handleMotionTarget, this);
    system_reset_sub_ = nh_.subscribe(HARDWARE_RESET_TOPIC, 10, &MobilityProcess::handleSystemReset, this);
    dropper_sub_ = nh_.subscribe(TIC_TAC_DROP_TOPIC, 10, &MobilityProcess::handleDropCommand, this);

    robot_state_pub_ = nh_.advertise<robot_pkg::RobotState>(ROBOT_STATE_TOPIC, 10);
    servo_command_pub_ = nh_.advertise<robot_pkg::ServoCommand>(SERVO_COMMAND_TOPIC, 100);

    current_state_.killed = true;
    move_mode_ = MoveMode::DONE;
    wheel_mode_ = WheelMode::UNKNOWN;
    last_transition_time_ = ros::Time::now();
    dist_to_travel_ = 0;
    straight_vel_ = 0;
    last_drop_time_ = ros::Time::now();
    current_drop_ = 0;
    requested_drop_ = 0;
}

MobilityProcess::~MobilityProcess() {}

int MobilityProcess::run() {
    mobility_update_timer_ = nh_.createTimer(ros::Duration(1.0 / MOBILITY_UPDATE_RATE_HZ), &MobilityProcess::updateMobility, this);
    ROS_INFO("Mobility set up");
    ros::spin();
    return 0;
}

void MobilityProcess::processKillswitch(std_msgs::Bool msg) {
    // Set killed
    current_state_.killed = msg.data;
    // Fast track to stop robot if killed
    if (current_state_.killed) {
        handleSystemReset(msg);
    }
}

void MobilityProcess::handleMotionTarget(robot_pkg::MotionTarget msg) {
    // If X or Y change, send to beginning
    if (current_target_.pos_x != msg.pos_x || 
        current_target_.pos_y != msg.pos_y) {
        
        current_state_.at_target = false;

        move_mode_ = MoveMode::TURNING_TO_POS;
        current_target_ = msg;
        dist_to_travel_ = distToTarget();
    }
    // If just angle has changed and we are already at the target, just make it yaw
    else if (current_target_.yaw != msg.yaw && move_mode_ == MoveMode::DONE) {
        current_state_.at_target = false;
        move_mode_ = MoveMode::TURNING_TO_ANGLE;
    }
    current_target_ = msg;

}

void MobilityProcess::handleSystemReset(std_msgs::Bool msg) {
    // If we got killed
    if (msg.data) {
        ROS_INFO("Got system reset");
        // Stop motor movements
        stopMoving();
        // We have no target currently
        move_mode_ = MoveMode::DONE;

        // Zero out wheel attributes
        wheel_mode_ = WheelMode::UNKNOWN;
        last_transition_time_ = ros::Time::now();
        dist_to_travel_ = 0;
        straight_vel_ = 0;

        // Zero out dropper state
        last_drop_time_ = ros::Time::now();
        current_drop_ = 0;
        requested_drop_ = 0;

        // Zero out current state
        current_state_.pos_x = 0;
        current_state_.pos_y = 0;
        current_state_.vel_x = 0;
        current_state_.vel_y = 0;
        current_state_.yaw = 0;

        // Zero out target position
        current_target_.pos_x = 0;
        current_target_.pos_y = 0;
        current_target_.yaw = 0;
    }
}

void MobilityProcess::handleDropCommand(std_msgs::Bool msg) {
    if (msg.data && requested_drop_ < MAX_TIC_TAC_DROPS) {
        requested_drop_++;
    }
}

void MobilityProcess::updateMobility(const ros::TimerEvent& time) {
    // If killed, don't do anything besides publish
    if (current_state_.killed) {
        ROS_INFO("Robot is killed, just publishing state");
        robot_state_pub_.publish(current_state_);
        return;
    }

    // Handle tic tac drop
    if (requested_drop_ > current_drop_ &&
        (ros::Time::now() - last_drop_time_).toSec() > DROP_WAIT_TIME) {
        
        current_drop_++;
        last_drop_time_ = ros::Time::now();
        ROS_INFO("Dropping tic tac %d", current_drop_);
    }
    setDropper(current_drop_);
    
    // Turn towards target
    if (move_mode_ == MoveMode::TURNING_TO_POS) {
        // We need to move somewhere
        current_state_.at_target = false;
        // If we are facing the right angle, stop
        if (abs(angleToTargetPos()) < MAX_ANG_ERROR) {
            ROS_INFO("Stopped turning");
            stopMoving();
            move_mode_ = MoveMode::MOVING;
        }
        // Make sure wheels are oriented to turn
        else if (wheel_mode_ != WheelMode::TURNING) {
            ROS_INFO("Moving wheels to turn");
            setWheelMode(WheelMode::TURNING);
        }
        else{
            // Command to turn to angle
            turnToAngle(angleToTargetPos());
            ROS_INFO("Turning to face target");
        }
    }

    // Move towards target
    if (move_mode_ == MoveMode::MOVING) {
        // We need to move somewhere
        current_state_.at_target = false;
        // If we are close, stop
        if (dist_to_travel_ <= 0) {
            ROS_INFO("Stopped moving forward");
            stopMoving();
            move_mode_ = MoveMode::TURNING_TO_ANGLE;
        }
        // Make sure wheels are oriented to go straight
        else if (wheel_mode_ != WheelMode::STRAIGHT) {
            ROS_INFO("Straightening wheels");
            setWheelMode(WheelMode::STRAIGHT);
        }
        else {
            // Command to go straight
            moveStraight(1);
            ROS_INFO("Moving forward");
        }
    }

    // Move towards target
    if (move_mode_ == MoveMode::TURNING_TO_ANGLE) {
        // We need to move somewhere
        current_state_.at_target = false;
        // If we are close, stop
        if (abs(angleToTargetAngle()) < MAX_ANG_ERROR) {
            ROS_INFO("Stopped turning");
            stopMoving();
            move_mode_ = MoveMode::DONE;
        }
        // Make sure wheels are oriented to turn
        else if (wheel_mode_ != WheelMode::TURNING) {
            ROS_INFO("Turning to face final yaw target");
            setWheelMode(WheelMode::TURNING);
        }
        else {
            // Command to turn
            turnToAngle(angleToTargetAngle());
            ROS_INFO("Turning to face yaw target");
        }
    }

    if (move_mode_ == MoveMode::DONE) {
        stopMoving();
        current_state_.at_target = true;
        ROS_INFO("At the target now, stop moving");
    }

    // Update all positions
    double time_diff = (time.current_real - time.last_real).toSec();
    current_state_.pos_x += time_diff * current_state_.vel_x;
    current_state_.pos_y += time_diff * current_state_.vel_y;
    current_state_.yaw += time_diff * current_state_.angular_vel;
    dist_to_travel_ -= time_diff * straight_vel_;

    // Publish robot state
    robot_state_pub_.publish(current_state_);
    
}

double MobilityProcess::angleToTargetPos() {
    double delta_x = current_target_.pos_x - current_state_.pos_x;
    double delta_y = current_target_.pos_y - current_state_.pos_y;
    double req_heading = atan2(delta_y, delta_x);
    double angle = req_heading - current_state_.yaw;
    ROS_INFO("Angle to target pos: %f", angle);
    while (angle > M_PI) {
        angle -= 2*M_PI;
    }
    while (angle < -M_PI) {
        angle += 2*M_PI;
    }
    return angle;
}

double MobilityProcess::angleToTargetAngle() {
    double angle = current_target_.yaw - current_state_.yaw;
    ROS_INFO("Angle to target angle: %f", angle);
    while (angle > M_PI) {
        angle -= 2*M_PI;
    }
    while (angle < -M_PI) {
        angle += 2*M_PI;
    }
    return angle;
}

double MobilityProcess::distToTarget() {
    double delta_x = current_target_.pos_x - current_state_.pos_x;
    double delta_y = current_target_.pos_y - current_state_.pos_y;
    double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    ROS_INFO("Dist away from target: %f", dist);
    return dist;
}

void MobilityProcess::setWheelMode(WheelMode mode) {
    sendWheelAngles(mode);

    if (wheel_mode_ == TRANSITIONING) {
        if (ros::Time::now() - last_transition_time_ >= ros::Duration(TRANSITION_WAIT_TIME)) {
            wheel_mode_ = mode;
        }
    }
    else if (wheel_mode_ != mode) {
        wheel_mode_ = TRANSITIONING;
        last_transition_time_ = ros::Time::now();
    }
}

void MobilityProcess::sendWheelAngles(WheelMode mode) {
    for (int i = 0; i < 3; i++) {
        robot_pkg::ServoCommand cmd;
        cmd.servo_id = PIVOT_IDS[i];
        switch(mode) {
            case WheelMode::STRAIGHT:
            cmd.value = STRAIGHT_SETPOINTS[i];
            break;
            case WheelMode::TURNING:
            cmd.value = TURNING_SETPOINTS[i];
            break;
            case WheelMode::STRAFE:
            cmd.value = STRAFE_SETPOINTS[i];
            break;
        }
        servo_command_pub_.publish(cmd);
    }
}

void MobilityProcess::stopMoving() {
    for (int i = 0; i < 3; i++) {
        robot_pkg::ServoCommand cmd;
        cmd.servo_id = WHEEL_IDS[i];
        cmd.value = 0;
        servo_command_pub_.publish(cmd);
    }
    current_state_.vel_x = 0;
    current_state_.vel_y = 0;
    current_state_.angular_vel = 0;
    straight_vel_ = 0;
}

void MobilityProcess::turnToAngle(double angle) {
    assert(wheel_mode_ == WheelMode::TURNING);
    double direction = (angle >= 0) ? 1 : -1;
    for (int i = 0; i < 3; i++) {
        robot_pkg::ServoCommand cmd;
        cmd.servo_id = WHEEL_IDS[i];
        cmd.value = MAX_TURN_SPEED  * TURN_TO_SPEED * SPEED_TO_POWER * TURN_DIRECTIONS[i] * direction;
        servo_command_pub_.publish(cmd);
    }
    current_state_.angular_vel = direction * MAX_TURN_SPEED;
}

void MobilityProcess::moveStraight(double dir) {
    assert(wheel_mode_ == WheelMode::STRAIGHT);
    for (int i = 0; i < 3; i++) {
        robot_pkg::ServoCommand cmd;
        cmd.servo_id = WHEEL_IDS[i];
        cmd.value = MAX_SPEED * SPEED_TO_POWER * STRAIGHT_DIRECTIONS[i];
        servo_command_pub_.publish(cmd);
    }
    current_state_.vel_x = MAX_SPEED * cos(current_state_.yaw);
    current_state_.vel_y = MAX_SPEED * sin(current_state_.yaw);
    straight_vel_ = MAX_SPEED;
}

void MobilityProcess::setDropper(int index) {
    robot_pkg::ServoCommand cmd;
    cmd.servo_id = DROPPER_ID;
    cmd.value = DROP_SETPOINTS[index];
    servo_command_pub_.publish(cmd);
}