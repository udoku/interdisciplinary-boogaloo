#ifndef _ROS_robot_pkg_UltrasonicPing_h
#define _ROS_robot_pkg_UltrasonicPing_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace robot_pkg
{

  class UltrasonicPing : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int64_t _sensor_id_type;
      _sensor_id_type sensor_id;
      typedef float _distance_type;
      _distance_type distance;
      enum { FRONT_LEFT =  0 };
      enum { FRONT_RIGHT =  1 };
      enum { LEFT =  2 };
      enum { RIGHT =  3 };
      enum { BACK_LEFT =  4 };
      enum { BACK_RIGHT =  5 };

    UltrasonicPing():
      header(),
      sensor_id(0),
      distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_sensor_id;
      u_sensor_id.real = this->sensor_id;
      *(outbuffer + offset + 0) = (u_sensor_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sensor_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sensor_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sensor_id.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sensor_id.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sensor_id.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sensor_id.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sensor_id.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sensor_id);
      offset += serializeAvrFloat64(outbuffer + offset, this->distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_sensor_id;
      u_sensor_id.base = 0;
      u_sensor_id.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sensor_id.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sensor_id.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sensor_id.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sensor_id.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sensor_id.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sensor_id.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sensor_id.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sensor_id = u_sensor_id.real;
      offset += sizeof(this->sensor_id);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->distance));
     return offset;
    }

    const char * getType(){ return "robot_pkg/UltrasonicPing"; };
    const char * getMD5(){ return "ece6a849d21f0f30e286fe4b3241a01c"; };

  };

}
#endif
