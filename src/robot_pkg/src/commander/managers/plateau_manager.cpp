#include "plateau_manager.hpp"

PlateauManager::PlateauManager() : message_("Hello World!") {}

string PlateauManager::getName() { return "Plateau"; }

bool PlateauManager::start() {
    ROS_INFO("Starting plateau");
    return call(bind(&PlateauManager::printMessage, this, 5));
}

// Dispatch while trying to get close to the bins
bool PlateauManager::printMessage(int n_times) {
    for (int i = 0; i < n_times; i++) {
        ROS_INFO("%s", message_.c_str());
    }
    return done(true);
}
