#include "finish_manager.hpp"

string FinishManager::getName() { return "Finish"; }

bool FinishManager::start() {
    ROS_FATAL("Task sequence finished!");
    System::setLedComplete();
    return done(true);
}
