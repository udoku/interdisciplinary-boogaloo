#include "mobility_manager.hpp"

MobilityManager::MobilityManager() : message_("Hello World!") {}

string MobilityManager::getName() { return "Mobility"; }

bool MobilityManager::start() {
    ROS_INFO("Starting mobility");
    return call(bind(&MobilityManager::printMessage, this, 5));
}

// Dispatch while trying to get close to the bins
bool MobilityManager::printMessage(int n_times) {
    for (int i = 0; i < n_times; i++) {
        ROS_INFO("%s", message_.c_str());
    }
    return done(true);
}
