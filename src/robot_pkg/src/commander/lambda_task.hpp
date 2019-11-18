#ifndef __LAMBDA_TASK__
#define __LAMBDA_TASK__

#include "common/common_header.hpp"
#include "task_manager_base.hpp"

/** This task makes it easy to turn any function (explicit, lambda, or bind-based) into a task
 * manager dynamically on the spot*/
class LambdaTask : public TaskManagerBase {
    string name_;
    function<bool()> f_;

  public:
    LambdaTask(string name, function<bool()> f);
    LambdaTask(string name, function<void()> f);
    virtual ~LambdaTask();
    virtual string getName();
    virtual bool run();
};

#endif /** #ifndef __LAMBDA_TASK__ **/
