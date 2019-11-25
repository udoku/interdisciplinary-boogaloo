#include "hardware_process.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, string(VISION_PROCESS));

    HardwareProcess* p = new HardwareProcess();
    p->run();
}

HardwareProcess::HardwareProcess() {
    hardware_reset_sub_ = nh_.subscribe(HARDWARE_RESET_TOPIC, 10, &HardwareProcess::processHardwareReset, this);
    servo_command_sub_ = nh_.subscribe(SERVO_COMMAND_TOPIC, 100, &HardwareProcess::processServoCommand, this);
    killswitch_pub_ = nh_.advertise<std_msgs::Bool>(KILLSWITCH_TOPIC, 10);
}

HardwareProcess::~HardwareProcess() {}

void HardwareProcess::processHardwareReset(std_msgs::Bool msg) {
    // TODO
}

void HardwareProcess::processServoCommand(robot_pkg::ServoCommand msg) {
    // TODO
}

void HardwareProcess::readKillswitch(const ros::TimerEvent& time) {
    // TODO actually read killswitch
    std_msgs::Bool msg;
    msg.data = false;
    killswitch_pub_.publish(msg);
}

int HardwareProcess::run() {
    killswitch_timer_ = nh_.createTimer(ros::Duration(1.0 / MOBILITY_UPDATE_RATE_HZ), &HardwareProcess::readKillswitch, this);
    
    ROS_INFO("Hardware set up");
    ros::spin();
}