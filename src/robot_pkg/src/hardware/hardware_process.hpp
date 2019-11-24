#ifndef __HARDWARE_PROCESS__
#define __HARDWARE_PROCESS__

#include "common/common_header.hpp"


class HardwareProcess {
  private:
    ros::NodeHandle nh_;

    ros::Publisher killswitch_pub_;

    ros::Subscriber hardware_reset_sub_;
    ros::Subscriber servo_command_sub_;

    ros::Timer killswitch_timer_;

  public:
    HardwareProcess();
    ~HardwareProcess();
    void processHardwareReset(std_msgs::Bool msg);
    void processServoCommand(robot_pkg::ServoCommand msg);

    void readKillswitch(const ros::TimerEvent& time);

    virtual int run();
};

#endif /** #ifndef __HARDWARE_PROCESS__ **/
