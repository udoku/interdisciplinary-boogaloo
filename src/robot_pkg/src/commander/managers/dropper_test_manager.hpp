#ifndef __DROPPER_TEST_MANAGER__
#define __DROPPER_TEST_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class DropperTestManager : public StateMachine {
  bool start();

  private:
    bool dropTicTac(int n_times);

  public:
    DropperTestManager();
    virtual string getName();
};

#endif
