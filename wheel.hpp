#ifndef _WHEEL_HPP_
#define _WHEEL_HPP_

#include "math.h"

class Wheel
{
public:
    Wheel(float radius, float wheel_rad, float beta_rad, float gamma_rad);

    void set_parameters(float radius, float wheel_rad, float beta_rad, float gamma_rad);

    float get_velocity(float x, float y, float theta);

private:
    float _radius;
    float _wheel_rad;
    float _position_x;
    float _position_y;
    float _beta_rad;
    float _gamma_rad;
    float _a_value;
    float _b_value;
    float _wheel_velocity_x;
    float _wheel_velocity_y;
    float _wheel_velocity_theta;
};

#endif