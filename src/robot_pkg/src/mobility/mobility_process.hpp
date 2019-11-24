#ifndef __MOBILITY_PROCESS__
#define __MOBILITY_PROCESS__

#include "common/common_header.hpp"


class MobilityProcess {
  private:
    ros::NodeHandle nh_;
    ros::Subscriber killswitch_sub_;
    ros::Subscriber motion_target_sub_;
    ros::Publisher robot_state_pub_;
    ros::Publisher servo_command_pub_;
    ros::Timer robot_state_timer_;

    robot_pkg::RobotState current_state_;

  public:
    MobilityProcess();
    ~MobilityProcess();
    virtual int run();

    void processKillswitch(std_msgs::Bool msg);
    void handleMotionTarget(robot_pkg::MotionTarget msg);

    void broadcastState(const ros::TimerEvent& time);
};

#endif /** #ifndef __MOBILITY_PROCESS__ **/
