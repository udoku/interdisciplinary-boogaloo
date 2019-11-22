#include "hardware_process.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, string(VISION_PROCESS));

    HardwareProcess* p = new HardwareProcess();
    p->run();
}

HardwareProcess::HardwareProcess() {
    hardware_reset_sub_ = nh_.subscribe(HARDWARE_RESET_TOPIC, 10, &HardwareProcess::processHardwareReset, this);
    servo_command_sub_ = nh_.subscribe(SERVO_COMMAND_TOPIC, 100, &HardwareProcess::processServoCommand, this);
}
HardwareProcess::~HardwareProcess() {}

void HardwareProcess::processHardwareReset(std_msgs::Bool msg) {
    // TODO
}

void HardwareProcess::processServoCommand(robot_pkg::ServoCommand msg) {
    // TODO
}

int HardwareProcess::run() {
    ros::spin();
}