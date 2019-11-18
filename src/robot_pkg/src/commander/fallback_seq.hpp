#ifndef __FALLBACK_SEQ__
#define __FALLBACK_SEQ__

#include "common/common_header.hpp"
#include "task_manager_base.hpp"
#include "task_seq.hpp"
#include "utils.hpp"

/** This class stores and runs arbtrary list of task managers in order. Except unlike
 * the TaskSeq class, this class will quit as soon as one task returns true. */
class FallbackSeq : public TaskManagerBase {
    vector<TimedTask> timed_tasks_;

  public:
    FallbackSeq(initializer_list<TimedTask> timed_tasks);
    FallbackSeq(initializer_list<TaskPtr> tasks);
    virtual string getName();
    virtual bool run();
};

#endif /** #ifndef __TASK_SEQ__ **/
