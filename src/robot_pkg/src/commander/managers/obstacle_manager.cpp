#include "obstacle_manager.hpp"

ObstacleManager::ObstacleManager() {}

string ObstacleManager::getName() { return "Obstacle"; }

bool ObstacleManager::start() {
    ROS_INFO("Starting obstacle");
    target_.pos_x = Motion::getCurrentState().pos_x;
    target_.pos_y = Motion::getCurrentState().pos_y;
    target_.yaw = Motion::getCurrentState().yaw;
    Vision::setDetector(robot_pkg::Detector::OBSTACLE_DETECTOR);
    return call(bind(&ObstacleManager::avoidObstacles, this));
}

// Dispatch while trying to get close to the bins
bool ObstacleManager::avoidObstacles() {
    vector<robot_pkg::Detection> dets;
    ros::Duration(1).sleep();

    while (true) {
        Vision::waitVisionData(robot_pkg::Detection::OBSTACLE, 10, &dets);

        // Create side point bubble
        robot_pkg::MotionTarget side_point_local;
        side_point_local.pos_x = .18;
        side_point_local.pos_y = -.18;
        robot_pkg::MotionTarget side_point = Motion::localToGlobal(side_point_local);
        double side_point_rad = .05;

        // Create front point bubble
        robot_pkg::MotionTarget front_point_local;
        front_point_local.pos_x = .12;
        front_point_local.pos_y = 0;
        robot_pkg::MotionTarget front_point = Motion::localToGlobal(front_point_local);
        double front_point_rad = .125;

        robot_pkg::MotionTarget local_target;
        // If det in front
        if (isDetAtPos(dets, front_point.pos_x, front_point.pos_y, front_point_rad)) {
            // Turn left
            ROS_INFO("Something in front! Turning left");
            local_target.yaw = M_PI/8;
        }
        else if (!isDetAtPos(dets, side_point.pos_x, side_point.pos_y, side_point_rad)) {
            // Turn right
            ROS_INFO("Nothing to right! Turning right");
            local_target.yaw = -M_PI/8;
        }
        else {
            // Go straight
            ROS_INFO("Vroom vroom");
            local_target.pos_x = side_point_local.pos_x;
        }
        
        target_ = Motion::localToGlobal(local_target);

        Motion::moveTo(target_);

        ros::Duration(.5).sleep();

        while (Motion::getCurrentState().at_target == false) {
            ros::Duration(.1).sleep();
        }

        ros::Duration(5).sleep();
    }

    ROS_INFO("Reached end of line!");
    return done(true);
}

bool ObstacleManager::isDetAtPos(vector<robot_pkg::Detection>& dets, double x_pos, double y_pos, double rad) {
    for (robot_pkg::Detection det : dets) {
        if (pow(x_pos-det.pos_x, 2) + pow(y_pos-det.pos_y, 2) < pow(rad, 2)) {
            return true;
        }
    }
    return false;
}
