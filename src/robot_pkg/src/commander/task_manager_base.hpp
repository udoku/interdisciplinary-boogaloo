#ifndef __TASK_MANAGER_BASE__
#define __TASK_MANAGER_BASE__

#include "common/common_header.hpp"
#include "utils.hpp"

/** All commanders should inherit from this class so they have access to these utilities. */
class TaskManagerBase {
  public:
    /**   If you ever dynamically allocate memory, it must be deallocated here (rather than inside
     * run) to avoid memory leaks since this thread could be canceled at any time */
    virtual ~TaskManagerBase(){};

    /** Should run the main code for the task. Return true for success */
    virtual bool run() = 0;

    /** Return a human-readable name for this task */
    virtual string getName() = 0;
};

class StateMachine : public TaskManagerBase {
    function<bool()> next_call_ = bind(&StateMachine::start, this);

  protected:
    /** The entry point to the state machine. Should be overloaded */
    virtual bool start() = 0;

    /** This is how you tell it which function to do next. Do something like:
     *  myFunc(int x){
     *      // Do some work
     *      if(needToDoMoreWork) return call(bind(myOtherFunction,this));
     *      else return done(true);
     *  } */
    bool call(function<bool()> f) __attribute__((warn_unused_result));
    bool done(bool status) __attribute__((warn_unused_result));

  public:
    bool run();
};

using TaskPtr = shared_ptr<TaskManagerBase>;

#endif /** __TASK_MANAGER_BASE__ **/
