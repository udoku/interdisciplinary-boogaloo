#include "hardware_process.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, string(VISION_PROCESS));

    HardwareProcess* p = new HardwareProcess();
    p->run();
}

HardwareProcess::HardwareProcess() {}
HardwareProcess::~HardwareProcess() {}

int HardwareProcess::run() {
    ros::spin();
}