#include "move_test_manager.hpp"

MoveTestManager::MoveTestManager() {}

string MoveTestManager::getName() { return "Move Test"; }

bool MoveTestManager::start() {
    ROS_INFO("Starting movement test");
    return call(bind(&MoveTestManager::testMove, this));
}

// Move to a point
bool MoveTestManager::testMove() {
    robot_pkg::MotionTarget target;
    target.pos_x = 2;
    target.pos_y = 0;
    target.yaw = 0;
    target.high_speed = true;
    target.reversed = false;


    Motion::moveTo(target);

    ros::Duration(0.5).sleep();

    while (Motion::getCurrentState().at_target == false) {
        ros::Duration(0.1).sleep();
    }

    ros::Duration(1).sleep(); 

    return done(true);
}
