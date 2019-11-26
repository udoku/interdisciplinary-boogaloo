#ifndef _ROS_robot_pkg_LedCommand_h
#define _ROS_robot_pkg_LedCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace robot_pkg
{

  class LedCommand : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _state_type;
      _state_type state;
      enum { DISABLED =  0 };
      enum { ARMED =  1 };
      enum { DISABLED_COMPLETE =  2 };

    LedCommand():
      header(),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "robot_pkg/LedCommand"; };
    const char * getMD5(){ return "053c6b7ea59bf9cacab6a9f12209462c"; };

  };

}
#endif
