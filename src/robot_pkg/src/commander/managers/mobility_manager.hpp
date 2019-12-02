#ifndef __MOBILITY_MANAGER__
#define __MOBILITY_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class MobilityManager : public StateMachine {
  bool start();

  private:
    const string message_;
    bool printMessage(int n_times);

  public:
    MobilityManager();
    virtual string getName();
};

#endif
