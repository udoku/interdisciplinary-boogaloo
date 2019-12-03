#ifndef __LINE_MANAGER__
#define __LINE_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

const double MAX_DIST = 1;

class LineManager : public StateMachine {
  bool start();

  private:
    double distance_;
    robot_pkg::MotionTarget target_;
    bool followLine();

  public:
    LineManager();
    virtual string getName();
};

#endif
