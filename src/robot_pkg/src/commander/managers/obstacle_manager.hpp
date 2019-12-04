#ifndef __OBSTACLE_MANAGER__
#define __OBSTACLE_MANAGER__

#include "../task_manager_base.hpp"
#include "common/common_header.hpp"

class ObstacleManager : public StateMachine {
  bool start();

  private:
    double distance_;
    robot_pkg::MotionTarget target_;
    bool wall_front_;
    bool wall_mid_;
    bool wall_back_;
    
    bool findWall();
    bool avoidObstacles();
    bool isDetAtPos(vector<robot_pkg::Detection>& dets, double x_pos, double y_pos, double rad);

  public:
    ObstacleManager();
    virtual string getName();
};

#endif
