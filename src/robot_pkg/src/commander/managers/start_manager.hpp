#ifndef __START_MANAGER__
#define __START_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class StartManager : public StateMachine {
  bool start();

  private:
    const string message_;
    bool printMessage(int n_times);

  public:
    StartManager();
    virtual string getName();
};

#endif
