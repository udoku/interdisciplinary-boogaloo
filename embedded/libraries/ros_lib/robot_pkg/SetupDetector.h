#ifndef _ROS_SERVICE_SetupDetector_h
#define _ROS_SERVICE_SetupDetector_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robot_pkg/Detector.h"

namespace robot_pkg
{

static const char SETUPDETECTOR[] = "robot_pkg/SetupDetector";

  class SetupDetectorRequest : public ros::Msg
  {
    public:
      typedef robot_pkg::Detector _detector_type;
      _detector_type detector;

    SetupDetectorRequest():
      detector()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->detector.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->detector.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETUPDETECTOR; };
    const char * getMD5(){ return "6b02056645b10421cb95ae3d38b8fce9"; };

  };

  class SetupDetectorResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetupDetectorResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETUPDETECTOR; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetupDetector {
    public:
    typedef SetupDetectorRequest Request;
    typedef SetupDetectorResponse Response;
  };

}
#endif
