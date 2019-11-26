#ifndef _ROS_robot_pkg_ServoCommand_h
#define _ROS_robot_pkg_ServoCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace robot_pkg
{

  class ServoCommand : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int64_t _servo_id_type;
      _servo_id_type servo_id;
      typedef float _value_type;
      _value_type value;

    ServoCommand():
      header(),
      servo_id(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_servo_id;
      u_servo_id.real = this->servo_id;
      *(outbuffer + offset + 0) = (u_servo_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_id.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_servo_id.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_servo_id.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_servo_id.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_servo_id.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->servo_id);
      offset += serializeAvrFloat64(outbuffer + offset, this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_servo_id;
      u_servo_id.base = 0;
      u_servo_id.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_id.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_id.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_id.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_servo_id.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_servo_id.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_servo_id.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_servo_id.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->servo_id = u_servo_id.real;
      offset += sizeof(this->servo_id);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->value));
     return offset;
    }

    const char * getType(){ return "robot_pkg/ServoCommand"; };
    const char * getMD5(){ return "ce322bf5dcf9c3f13c51b1597ca03706"; };

  };

}
#endif
