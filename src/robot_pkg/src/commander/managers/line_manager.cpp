#include "line_manager.hpp"

LineManager::LineManager() : distance_(0) {}

string LineManager::getName() { return "Line"; }

bool LineManager::start() {
    ROS_INFO("Starting line");
    target_.pos_x = Motion::getCurrentState().pos_x;
    target_.pos_y = Motion::getCurrentState().pos_y;
    target_.yaw = Motion::getCurrentState().yaw;
    Vision::setDetector(robot_pkg::Detector::LINE_DETECTOR);
    return call(bind(&LineManager::followLine, this));
}

bool LineManager::followLine() {
    vector<robot_pkg::Detection> dets;
    ros::Duration(1).sleep();

    while (distance_ < MAX_DIST) {
        Vision::waitVisionData(robot_pkg::Detection::LINE, 10, &dets, 1, 4);

        // No dets, cry. TODO search for line
        if (dets.size() == 0) {
            ROS_WARN("No line found, failing task");
            return done(false);
        }

        robot_pkg::Detection det = dets[0];

        distance_ += sqrt(pow(target_.pos_x-det.pos_x, 2) + pow(target_.pos_y-det.pos_y, 2));

        target_.pos_x = det.pos_x;
        target_.pos_y = det.pos_y;
        target_.yaw = det.orientation;

        Motion::moveTo(target_);

        ros::Duration(.5).sleep();

        while (Motion::getCurrentState().at_target == false) {
            ros::Duration(.1).sleep();
        }

        ros::Duration(2).sleep();
    }

    ROS_INFO("Reached end of line!");

    // Move forward a bit to go to next table
    robot_pkg::MotionTarget local_target;
    local_target.pos_x = .2;
    local_target.pos_y = 0;
    target_ = Motion::localToGlobal(local_target);
    Motion::moveTo(target_);

    ros::Duration(.5).sleep();

    while (Motion::getCurrentState().at_target == false) {
        ros::Duration(.1).sleep();
    }

    return done(true);
}
