#ifndef __MOVE_TEST_MANAGER__
#define __MOVE_TEST_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class MoveTestManager : public StateMachine {
  bool start();

  private:
    bool testMove();

  public:
    MoveTestManager();
    virtual string getName();
};

#endif
