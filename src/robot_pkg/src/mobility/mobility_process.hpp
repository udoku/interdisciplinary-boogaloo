#ifndef __MOBILITY_PROCESS__
#define __MOBILITY_PROCESS__

#include "common/common_header.hpp"

class MobilityProcess {
  private:

    enum WheelMode {STRAIGHT, STRAFE, TURNING, TRANSITIONING, UNKNOWN};
    enum MoveMode {TURNING_TO_POS, MOVING, TURNING_TO_ANGLE, DONE};

    ros::NodeHandle nh_;
    ros::Subscriber killswitch_sub_;
    ros::Subscriber motion_target_sub_;
    ros::Subscriber system_reset_sub_;
    ros::Subscriber dropper_sub_;
    ros::Publisher robot_state_pub_;
    ros::Publisher servo_command_pub_;
    ros::Timer mobility_update_timer_;

    robot_pkg::RobotState current_state_;
    robot_pkg::MotionTarget current_target_;

    MoveMode move_mode_;
    WheelMode wheel_mode_;
    ros::Time last_transition_time_;
    double dist_to_travel_;
    double straight_vel_;

    ros::Time last_drop_time_;
    int current_drop_;
    int requested_drop_;

    double angleToTargetPos();
    double angleToTargetAngle();

    double distToTarget();

    void setWheelMode(WheelMode mode);
    void sendWheelAngles(WheelMode mode);

    void stopMoving();
    void turnToAngle(double angle);
    void moveStraight();

    void setDropper(int index);

  public:
    MobilityProcess();
    ~MobilityProcess();
    virtual int run();

    void processKillswitch(std_msgs::Bool msg);
    void handleMotionTarget(robot_pkg::MotionTarget msg);
    void handleSystemReset(std_msgs::Bool msg);
    void handleDropCommand(std_msgs::Bool msg);

    void updateMobility(const ros::TimerEvent& time);
};

#endif /** #ifndef __MOBILITY_PROCESS__ **/
