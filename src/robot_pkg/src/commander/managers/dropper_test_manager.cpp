#include "dropper_test_manager.hpp"

DropperTestManager::DropperTestManager() : {}

string DropperTestManager::getName() { return "Dropper Test"; }

bool DropperTestManager::start() {
    ROS_INFO("Starting dropper test");
    return call(bind(&DropperTestManager::dropTicTac, this, 4, 1.0));
}

// Dispatch a number of DropTicTac messages
bool DropperTestManager::dropTicTac(int n_times) {
    if (n_times > 0) {
        // Use utils to send DropTicTac message (which doesn't exist yet)
        ros::Duration(delay).sleep();
        return call(bind(&DropperTestManager::dropTicTac, this, n_times - 1));
    }
    else {
        return done(true);
    }
}
