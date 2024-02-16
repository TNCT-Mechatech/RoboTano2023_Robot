#include "Wheel.hpp"

Wheel::Wheel(float radius, float wheel_rad, float beta_rad, float gamma_rad) {
    set_parameters(radius, wheel_rad, beta_rad, gamma_rad);
}

/**
 * @brief set the parameters of the wheel
 * @param radius
 * @param wheel_rad
 * @param beta_rad
 * @param gamma_rad
 */
void Wheel::set_parameters(float radius, float wheel_rad, float beta_rad, float gamma_rad) {
    _radius = radius;
    _wheel_rad = wheel_rad;
    _beta_rad = beta_rad;
    _gamma_rad = gamma_rad;

    //  position
    _position_x = _radius * cos(_wheel_rad);
    _position_y = _radius * sin(_wheel_rad);

    //  value replacement
    _a_value = cos(_beta_rad) - sin(_beta_rad) * tan(_gamma_rad);
    _b_value = sin(_beta_rad) + cos(_beta_rad) * tan(_gamma_rad);
}

/**
 * @brief calculate the velocity of the wheel
 * @param x [m/s]
 * @param y [m/s]
 * @param theta [rad/s]
 * @return velocity of wheel [m/s]
 */
float Wheel::get_velocity(float x, float y, float theta) {
    _wheel_velocity_theta = (_b_value * _position_x - _a_value * _position_y) * theta;
    _wheel_velocity_x = _a_value * x;
    _wheel_velocity_y = _b_value * y;

    return _wheel_velocity_x + _wheel_velocity_y + _wheel_velocity_theta;
}