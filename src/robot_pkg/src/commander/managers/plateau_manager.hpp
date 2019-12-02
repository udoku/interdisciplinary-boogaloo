#ifndef __PLATEAU_MANAGER__
#define __PLATEAU_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class PlateauManager : public StateMachine {
  bool start();

  private:
    const string message_;
    bool printMessage(int n_times);

  public:
    PlateauManager();
    virtual string getName();
};

#endif
