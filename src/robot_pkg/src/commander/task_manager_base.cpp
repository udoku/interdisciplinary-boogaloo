#include "task_manager_base.hpp"

bool StateMachine::run() {
    bool result = true;
    while (next_call_ != nullptr)
        result = next_call_();
    return result;
}

bool StateMachine::call(function<bool()> f) {
    next_call_ = f;
    return true;
}

bool StateMachine::done(bool status) {
    next_call_ = nullptr;
    return status;
}
