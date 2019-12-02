#ifndef __OBSTACLE_MANAGER__
#define __OBSTACLE_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class ObstacleManager : public StateMachine {
  bool start();

  private:
    const string message_;
    bool printMessage(int n_times);

  public:
    ObstacleManager();
    virtual string getName();
};

#endif
