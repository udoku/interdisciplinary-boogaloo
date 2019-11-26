#ifndef _ROS_robot_pkg_Detector_h
#define _ROS_robot_pkg_Detector_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_pkg
{

  class Detector : public ros::Msg
  {
    public:
      typedef int32_t _detector_type_type;
      _detector_type_type detector_type;
      typedef int32_t _detector_mode_type;
      _detector_mode_type detector_mode;
      enum { NO_DETECTOR =  0 };
      enum { DEMO_DETECTOR =  1 };
      enum { DEFAULT_MODE =  0 };

    Detector():
      detector_type(0),
      detector_mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_detector_type;
      u_detector_type.real = this->detector_type;
      *(outbuffer + offset + 0) = (u_detector_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_detector_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_detector_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_detector_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->detector_type);
      union {
        int32_t real;
        uint32_t base;
      } u_detector_mode;
      u_detector_mode.real = this->detector_mode;
      *(outbuffer + offset + 0) = (u_detector_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_detector_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_detector_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_detector_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->detector_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_detector_type;
      u_detector_type.base = 0;
      u_detector_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_detector_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_detector_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_detector_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->detector_type = u_detector_type.real;
      offset += sizeof(this->detector_type);
      union {
        int32_t real;
        uint32_t base;
      } u_detector_mode;
      u_detector_mode.base = 0;
      u_detector_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_detector_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_detector_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_detector_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->detector_mode = u_detector_mode.real;
      offset += sizeof(this->detector_mode);
     return offset;
    }

    const char * getType(){ return "robot_pkg/Detector"; };
    const char * getMD5(){ return "4b246b713387534e8bdae9cbcefd6045"; };

  };

}
#endif
