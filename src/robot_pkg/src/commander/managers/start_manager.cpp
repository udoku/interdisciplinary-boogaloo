#include "start_manager.hpp"

StartManager::StartManager() : message_("Hello World!") {}

string StartManager::getName() { return "Start"; }

bool StartManager::start() {
    ROS_INFO("Starting starting table");
    return call(bind(&StartManager::printMessage, this, 5));
}

// Dispatch while trying to get close to the bins
bool StartManager::printMessage(int n_times) {
    for (int i = 0; i < n_times; i++) {
        ROS_INFO("%s", message_.c_str());
    }
    return done(true);
}
