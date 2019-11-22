#include "demo_manager.hpp"

DemoManager::DemoManager() : message_("Hello World!") {}

string DemoManager::getName() { return "Demo"; }

bool DemoManager::start() {
    ROS_INFO("Starting demo");
    return call(bind(&DemoManager::printMessage, this, 5));
}

// Dispatch while trying to get close to the bins
bool DemoManager::printMessage(int n_times) {
    for (int i = 0; i < n_times; i++) {
        ROS_INFO("%s", message_.c_str());
    }
    return done(true);
}