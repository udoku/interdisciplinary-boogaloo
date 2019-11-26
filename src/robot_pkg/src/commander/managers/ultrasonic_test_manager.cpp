#include "ultrasonic_test_manager.hpp"

UltrasonicTestManager::UltrasonicTestManager() {}

string UltrasonicTestManager::getName() { return "Ultrasonic Test"; }

bool UltrasonicTestManager::start() {
    ROS_INFO("Starting ultrasonic test");
    return call(bind(&UltrasonicTestManager::printUltrasonics, this));
}

// Move to a point
bool UltrasonicTestManager::printUltrasonics() {
    ros::Time start = ros::Time::now();
    while ((ros::Time::now() - start).toSec() < 30) {
        Ultrasonics ultrasonics = System::getUltrasonicData();
        ROS_INFO("Ultrasonic data: \n%f %f \n%f %f \n%f %f",
            ultrasonics.front_left, ultrasonics.front_right,
            ultrasonics.left, ultrasonics.right,
            ultrasonics.back_left, ultrasonics.back_right);
        ros::Duration(0.5).sleep();
    }
    return done(true);
}
