#ifndef __HARDWARE_PROCESS__
#define __HARDWARE_PROCESS__

#include "common/common_header.hpp"


class HardwareProcess {
  private:
    ros::NodeHandle nh_;

  public:
    HardwareProcess();
    ~HardwareProcess();
    virtual int run();
};

#endif /** #ifndef __HARDWARE_PROCESS__ **/
