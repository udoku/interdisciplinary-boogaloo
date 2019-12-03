#include "obstacle_manager.hpp"

ObstacleManager::ObstacleManager() : message_("Hello World!") {}

string ObstacleManager::getName() { return "Obstacle"; }

bool ObstacleManager::start() {
    ROS_INFO("Starting obstacle");
    Vision::setDetector(robot_pkg::Detector::OBSTACLE_DETECTOR);
    return call(bind(&ObstacleManager::printMessage, this, 5));
}

// Dispatch while trying to get close to the bins
bool ObstacleManager::printMessage(int n_times) {
    for (int i = 0; i < n_times; i++) {
        ROS_INFO("%s", message_.c_str());
    }
    return done(true);
}
