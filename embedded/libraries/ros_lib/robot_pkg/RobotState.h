#ifndef _ROS_robot_pkg_RobotState_h
#define _ROS_robot_pkg_RobotState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace robot_pkg
{

  class RobotState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _killed_type;
      _killed_type killed;
      typedef float _pos_x_type;
      _pos_x_type pos_x;
      typedef float _pos_y_type;
      _pos_y_type pos_y;
      typedef float _vel_x_type;
      _vel_x_type vel_x;
      typedef float _vel_y_type;
      _vel_y_type vel_y;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _angular_vel_type;
      _angular_vel_type angular_vel;
      typedef bool _at_target_type;
      _at_target_type at_target;

    RobotState():
      header(),
      killed(0),
      pos_x(0),
      pos_y(0),
      vel_x(0),
      vel_y(0),
      yaw(0),
      pitch(0),
      roll(0),
      angular_vel(0),
      at_target(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_killed;
      u_killed.real = this->killed;
      *(outbuffer + offset + 0) = (u_killed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->killed);
      offset += serializeAvrFloat64(outbuffer + offset, this->pos_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->pos_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->vel_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->vel_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->angular_vel);
      union {
        bool real;
        uint8_t base;
      } u_at_target;
      u_at_target.real = this->at_target;
      *(outbuffer + offset + 0) = (u_at_target.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->at_target);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_killed;
      u_killed.base = 0;
      u_killed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->killed = u_killed.real;
      offset += sizeof(this->killed);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pos_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pos_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vel_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vel_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angular_vel));
      union {
        bool real;
        uint8_t base;
      } u_at_target;
      u_at_target.base = 0;
      u_at_target.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->at_target = u_at_target.real;
      offset += sizeof(this->at_target);
     return offset;
    }

    const char * getType(){ return "robot_pkg/RobotState"; };
    const char * getMD5(){ return "6cdd696d7b429fb8befd4c259a3463b2"; };

  };

}
#endif
