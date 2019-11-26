#ifndef _ROS_SERVICE_RunDetector_h
#define _ROS_SERVICE_RunDetector_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Image.h"
#include "robot_pkg/Detections.h"
#include "robot_pkg/Detector.h"
#include "robot_pkg/Frame.h"

namespace robot_pkg
{

static const char RUNDETECTOR[] = "robot_pkg/RunDetector";

  class RunDetectorRequest : public ros::Msg
  {
    public:
      typedef robot_pkg::Frame _frame_type;
      _frame_type frame;
      typedef robot_pkg::Detector _detector_type;
      _detector_type detector;

    RunDetectorRequest():
      frame(),
      detector()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->frame.serialize(outbuffer + offset);
      offset += this->detector.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->frame.deserialize(inbuffer + offset);
      offset += this->detector.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return RUNDETECTOR; };
    const char * getMD5(){ return "c44c344dcfeaacfc67cd1c103bf19dec"; };

  };

  class RunDetectorResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef sensor_msgs::Image _feedback_type;
      _feedback_type feedback;
      typedef robot_pkg::Detections _detections_type;
      _detections_type detections;

    RunDetectorResponse():
      success(0),
      feedback(),
      detections()
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
      offset += this->feedback.serialize(outbuffer + offset);
      offset += this->detections.serialize(outbuffer + offset);
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
      offset += this->feedback.deserialize(inbuffer + offset);
      offset += this->detections.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return RUNDETECTOR; };
    const char * getMD5(){ return "6abbf26476a9e63e9ad30862a318e00c"; };

  };

  class RunDetector {
    public:
    typedef RunDetectorRequest Request;
    typedef RunDetectorResponse Response;
  };

}
#endif
