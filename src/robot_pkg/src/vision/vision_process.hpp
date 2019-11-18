#ifndef __VISION_PROCESS__
#define __VISION_PROCESS__

#include "common/common_header.hpp"

class VisionProcess {
  private:
    ros::NodeHandle nh_;

  public:
    VisionProcess();
    ~VisionProcess();
    virtual int run();
};

#endif /** #ifndef __VISION_PROCESS__ **/
