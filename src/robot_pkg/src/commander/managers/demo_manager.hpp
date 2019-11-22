#ifndef __DEMO_MANAGER__
#define __DEMO_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class DemoManager : public StateMachine {
  bool start();

  private:
    const string message_;
    bool printMessage(int n_times);

  public:
    DemoManager();
    virtual string getName();
};

#endif