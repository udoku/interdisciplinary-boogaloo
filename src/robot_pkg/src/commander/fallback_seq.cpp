#include "fallback_seq.hpp"

FallbackSeq::FallbackSeq(initializer_list<TimedTask> timed_tasks) : timed_tasks_(timed_tasks) {}
FallbackSeq::FallbackSeq(initializer_list<TaskPtr> tasks) {
    for (auto& task : tasks)
        timed_tasks_.push_back({-1, task});
}

string FallbackSeq::getName() { return "Fallback Sequence"; }

bool FallbackSeq::run() {
    for (auto& t : timed_tasks_) {
        if (Timeout::runWithTimeout(bind(&TaskManagerBase::run, t.task_), t.timeout_,
                                    t.task_->getName())) {
            return true;
        }
    }

    return false;
}
