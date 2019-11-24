#ifndef __TASK_SEQ__
#define __TASK_SEQ__

#include "common/common_header.hpp"
#include "task_manager_base.hpp"

struct TimedTask {
    int timeout_;  // Seconds (-1 => untimed)
    TaskPtr task_; // Taks to run
};

/** This class stores and runs arbtrary list of task managers in order*/
class TaskSeq : public TaskManagerBase {
    vector<TimedTask> timed_tasks_;

  public:
    TaskSeq(initializer_list<TimedTask> timed_tasks);
    TaskSeq(initializer_list<TaskPtr> tasks);
    virtual string getName();
    virtual bool run();
};

#endif /** #ifndef __TASK_SEQ__ **/
