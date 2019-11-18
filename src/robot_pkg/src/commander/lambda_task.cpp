#include "lambda_task.hpp"

bool voidToBoolWrapper(function<void()> f) {
    f();
    return true;
}

LambdaTask::LambdaTask(string name, function<bool()> f) : name_(name), f_(f) {}

LambdaTask::LambdaTask(string name, function<void()> f)
    : name_(name), f_(bind(voidToBoolWrapper, f)) {}

LambdaTask::~LambdaTask() {}

string LambdaTask::getName() { return name_; }

bool LambdaTask::run() { return f_(); }
