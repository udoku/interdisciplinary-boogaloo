#include "line_manager.hpp"

LineManager::LineManager() : message_("Hello World!") {}

string LineManager::getName() { return "Line"; }

bool LineManager::start() {
    ROS_INFO("Starting line");
    return call(bind(&LineManager::printMessage, this, 5));
}

// Dispatch while trying to get close to the bins
bool LineManager::printMessage(int n_times) {
    for (int i = 0; i < n_times; i++) {
        ROS_INFO("%s", message_.c_str());
    }
    return done(true);
}
