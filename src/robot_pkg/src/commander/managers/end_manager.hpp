#ifndef __END_MANAGER__
#define __END_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class EndManager : public StateMachine {
  bool start();

  private:
    const string message_;
    bool printMessage(int n_times);

  public:
    EndManager();
    virtual string getName();
};

#endif
