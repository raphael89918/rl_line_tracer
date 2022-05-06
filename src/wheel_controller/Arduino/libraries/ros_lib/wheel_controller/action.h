#ifndef _ROS_wheel_controller_action_h
#define _ROS_wheel_controller_action_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace wheel_controller
{

  class action : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _angular_action_type;
      _angular_action_type angular_action;
      typedef int8_t _linear_action_type;
      _linear_action_type linear_action;
      typedef bool _revert_type;
      _revert_type revert;

    action():
      header(),
      angular_action(0),
      linear_action(0),
      revert(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_angular_action;
      u_angular_action.real = this->angular_action;
      *(outbuffer + offset + 0) = (u_angular_action.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angular_action);
      union {
        int8_t real;
        uint8_t base;
      } u_linear_action;
      u_linear_action.real = this->linear_action;
      *(outbuffer + offset + 0) = (u_linear_action.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->linear_action);
      union {
        bool real;
        uint8_t base;
      } u_revert;
      u_revert.real = this->revert;
      *(outbuffer + offset + 0) = (u_revert.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->revert);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_angular_action;
      u_angular_action.base = 0;
      u_angular_action.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->angular_action = u_angular_action.real;
      offset += sizeof(this->angular_action);
      union {
        int8_t real;
        uint8_t base;
      } u_linear_action;
      u_linear_action.base = 0;
      u_linear_action.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->linear_action = u_linear_action.real;
      offset += sizeof(this->linear_action);
      union {
        bool real;
        uint8_t base;
      } u_revert;
      u_revert.base = 0;
      u_revert.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->revert = u_revert.real;
      offset += sizeof(this->revert);
     return offset;
    }

    virtual const char * getType() override { return "wheel_controller/action"; };
    virtual const char * getMD5() override { return "b7a37ef7ced75025cba4dd06be5f26ba"; };

  };

}
#endif
