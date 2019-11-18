#include "vision_process.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, string(VISION_PROCESS));

    VisionProcess* p = new VisionProcess();
    p->run();
}

VisionProcess::VisionProcess() {}
VisionProcess::~VisionProcess() {}

int VisionProcess::run() {}