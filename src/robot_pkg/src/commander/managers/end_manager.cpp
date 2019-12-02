#include "end_manager.hpp"

EndManager::EndManager() : message_("Hello World!") {}

string EndManager::getName() { return "End"; }

bool EndManager::start() {
    ROS_INFO("Starting end");
    return call(bind(&EndManager::printMessage, this, 5));
}

// Dispatch while trying to get close to the bins
bool EndManager::printMessage(int n_times) {
    for (int i = 0; i < n_times; i++) {
        ROS_INFO("%s", message_.c_str());
    }
    return done(true);
}
