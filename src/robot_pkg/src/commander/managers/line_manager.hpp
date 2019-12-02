#ifndef __LINE_MANAGER__
#define __LINE_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class LineManager : public StateMachine {
  bool start();

  private:
    const string message_;
    bool printMessage(int n_times);

  public:
    LineManager();
    virtual string getName();
};

#endif
