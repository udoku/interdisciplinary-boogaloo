#ifndef __PLATEAU_MANAGER__
#define __PLATEAU_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class PlateauManager : public StateMachine {
  bool start();

  private:
    robot_pkg::MotionTarget target_;
    bool go();

  public:
    PlateauManager();
    virtual string getName();
};

#endif
