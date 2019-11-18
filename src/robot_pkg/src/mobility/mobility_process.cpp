#include "mobility_process.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, string(VISION_PROCESS));

    MobilityProcess* p = new MobilityProcess();
    p->run();
}

MobilityProcess::MobilityProcess() {
    killswitch_sub_ = nh_.subscribe(KILLSWITCH_TOPIC, 1, &MobilityProcess::processKillswitch, this);
    motion_target_sub_ = nh_.subscribe(MOTION_TARGET_TOPIC, 1, &MobilityProcess::handleMotionTarget, this);
    robot_state_pub_ = nh_.advertise<robot_pkg::RobotState>(ROBOT_STATE_TOPIC, 1);
    current_state_.killed = true;
}

MobilityProcess::~MobilityProcess() {}

int MobilityProcess::run() {
    robot_state_timer_ = nh_.createTimer(ros::Duration(1.0 / ROBOT_STATE_UPDATE_RATE_HZ), &MobilityProcess::broadcastState, this);
    ros::spin();
}

void MobilityProcess::processKillswitch(std_msgs::Bool msg) {
    current_state_.killed = msg.data;
}

void MobilityProcess::handleMotionTarget(std_msgs::Bool msg) {
}

void MobilityProcess::broadcastState(const ros::TimerEvent& time) {
    robot_state_pub_.publish(current_state_);
}