#include "plateau_manager.hpp"

PlateauManager::PlateauManager() {}

string PlateauManager::getName() { return "Plateau"; }

bool PlateauManager::start() {
    ROS_INFO("Starting plateau");
    return call(bind(&PlateauManager::go, this));
}

// Dispatch while trying to get close to the bins
bool PlateauManager::go() {
    robot_pkg::MotionTarget local_target;
    local_target.pos_x = 1.2;
    target_ = Motion::localToGlobal(local_target);
    target_.high_speed = true;

    Motion::moveTo(target_);

    ros::Duration(.5).sleep();

    while (Motion::getCurrentState().at_target == false) {
        ros::Duration(.1).sleep();
    }
    ros::Duration(2).sleep();

    Actuators::drop();
    target_.high_speed = false;

    local_target.pos_x = 0.5;
    target_ = Motion::localToGlobal(local_target);

    Motion::moveTo(target_);

    ros::Duration(.5).sleep();

    while (Motion::getCurrentState().at_target == false) {
        ros::Duration(.1).sleep();
    }
    ros::Duration(2).sleep();
}
