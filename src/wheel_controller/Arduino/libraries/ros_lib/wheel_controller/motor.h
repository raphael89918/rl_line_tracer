#ifndef _ROS_wheel_controller_motor_h
#define _ROS_wheel_controller_motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace wheel_controller
{

  class motor : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _FL_type;
      _FL_type FL;
      typedef uint8_t _FR_type;
      _FR_type FR;
      typedef uint8_t _BL_type;
      _BL_type BL;
      typedef uint8_t _BR_type;
      _BR_type BR;
      typedef bool _FL_DIR_type;
      _FL_DIR_type FL_DIR;
      typedef bool _FR_DIR_type;
      _FR_DIR_type FR_DIR;
      typedef bool _BL_DIR_type;
      _BL_DIR_type BL_DIR;
      typedef bool _BR_DIR_type;
      _BR_DIR_type BR_DIR;

    motor():
      header(),
      FL(0),
      FR(0),
      BL(0),
      BR(0),
      FL_DIR(0),
      FR_DIR(0),
      BL_DIR(0),
      BR_DIR(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->FL >> (8 * 0)) & 0xFF;
      offset += sizeof(this->FL);
      *(outbuffer + offset + 0) = (this->FR >> (8 * 0)) & 0xFF;
      offset += sizeof(this->FR);
      *(outbuffer + offset + 0) = (this->BL >> (8 * 0)) & 0xFF;
      offset += sizeof(this->BL);
      *(outbuffer + offset + 0) = (this->BR >> (8 * 0)) & 0xFF;
      offset += sizeof(this->BR);
      union {
        bool real;
        uint8_t base;
      } u_FL_DIR;
      u_FL_DIR.real = this->FL_DIR;
      *(outbuffer + offset + 0) = (u_FL_DIR.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->FL_DIR);
      union {
        bool real;
        uint8_t base;
      } u_FR_DIR;
      u_FR_DIR.real = this->FR_DIR;
      *(outbuffer + offset + 0) = (u_FR_DIR.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->FR_DIR);
      union {
        bool real;
        uint8_t base;
      } u_BL_DIR;
      u_BL_DIR.real = this->BL_DIR;
      *(outbuffer + offset + 0) = (u_BL_DIR.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->BL_DIR);
      union {
        bool real;
        uint8_t base;
      } u_BR_DIR;
      u_BR_DIR.real = this->BR_DIR;
      *(outbuffer + offset + 0) = (u_BR_DIR.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->BR_DIR);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->FL =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->FL);
      this->FR =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->FR);
      this->BL =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->BL);
      this->BR =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->BR);
      union {
        bool real;
        uint8_t base;
      } u_FL_DIR;
      u_FL_DIR.base = 0;
      u_FL_DIR.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->FL_DIR = u_FL_DIR.real;
      offset += sizeof(this->FL_DIR);
      union {
        bool real;
        uint8_t base;
      } u_FR_DIR;
      u_FR_DIR.base = 0;
      u_FR_DIR.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->FR_DIR = u_FR_DIR.real;
      offset += sizeof(this->FR_DIR);
      union {
        bool real;
        uint8_t base;
      } u_BL_DIR;
      u_BL_DIR.base = 0;
      u_BL_DIR.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->BL_DIR = u_BL_DIR.real;
      offset += sizeof(this->BL_DIR);
      union {
        bool real;
        uint8_t base;
      } u_BR_DIR;
      u_BR_DIR.base = 0;
      u_BR_DIR.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->BR_DIR = u_BR_DIR.real;
      offset += sizeof(this->BR_DIR);
     return offset;
    }

    virtual const char * getType() override { return "wheel_controller/motor"; };
    virtual const char * getMD5() override { return "35a56bb2e8e73cf810af409cd4072b69"; };

  };

}
#endif
