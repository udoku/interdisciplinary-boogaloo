#ifndef __FINISH_MANAGER__
#define __FINISH_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class FinishManager : public StateMachine {
  bool start();

  private:

  public:
    virtual string getName();
};

#endif
