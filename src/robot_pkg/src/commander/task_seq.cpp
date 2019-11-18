#include "task_seq.hpp"

TaskSeq::TaskSeq(initializer_list<TimedTask> timed_tasks) : timed_tasks_(timed_tasks) {}
TaskSeq::TaskSeq(initializer_list<TaskPtr> tasks) {
    for (auto& task : tasks)
        timed_tasks_.push_back({-1, task});
}

string TaskSeq::getName() { return "Task Sequence"; }

bool TaskSeq::run() {
    bool ret_val = true;

    for (auto& t : timed_tasks_) {
        ret_val &= Timeout::runWithTimeout(bind(&TaskManagerBase::run, t.task_), t.timeout_,
                                          t.task_->getName());
    }

    return ret_val;
}
