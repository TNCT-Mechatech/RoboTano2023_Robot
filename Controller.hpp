#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include "Message.hpp"
#include <cstdint>

typedef struct ControlType {
  int8_t joystick_x;
  int8_t joystick_y;
  int8_t joystick_turn;
  int8_t moter_speed;
  int8_t roller_status;
  bool shoot_bottom;
  bool arm_up;
  bool arm_down;
  bool hand_status;
} control_t;

//  create message
typedef sb::Message<control_t> Control;

#endif