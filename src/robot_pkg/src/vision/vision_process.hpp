#ifndef __VISION_PROCESS__
#define __VISION_PROCESS__

#include "common/common_header.hpp"

class VisionProcess {
  private:
    ros::NodeHandle nh_;

    int detector_mode_ = 0;
    robot_pkg::Detector current_detector_;
    robot_pkg::Detector new_detector_;
    robot_pkg::Frame current_frame_;

    ros::Subscriber detector_sub_;
    ros::Subscriber images_sub_;
    ros::Publisher detection_image_pub_;
    ros::Publisher detections_pub_;

    ros::Timer detector_timer_;

    void runDetectors(const ros::TimerEvent& time);
    void changeDetector(const robot_pkg::Detector::ConstPtr& msg);
    void processImage(const robot_pkg::Frame::ConstPtr& msg);
    void checkSwitchDetectors();

  public:
    VisionProcess();
    ~VisionProcess();
    virtual int run();
};

#endif /** #ifndef __VISION_PROCESS__ **/
