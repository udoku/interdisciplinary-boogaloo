#include "move_test_manager.hpp"

MoveTestManager::MoveTestManager() {}

string MoveTestManager::getName() { return "Move Test"; }

bool MoveTestManager::start() {
    ROS_INFO("Starting dropper test");
    return call(bind(&MoveTestManager::testMove, this));
}

// Move ti a point
bool MoveTestManager::testMove() {
    robot_pkg::MotionTarget target;
    target.pos_x = 1;
    target.pos_y = 1;
    target.yaw = 0;

    Motion::moveTo(target);

    ros::Duration(0.5).sleep();

    while (Motion::getCurrentState().at_target == false) {
        ros::Duration(0.1).sleep();
    }

    target.pos_x = -1;
    target.yaw = M_PI;
    Motion::moveTo(target);

    ros::Duration(0.5).sleep();

    while (Motion::getCurrentState().at_target == false) {
        ros::Duration(0.1).sleep();
    }
    return done(true);
}
