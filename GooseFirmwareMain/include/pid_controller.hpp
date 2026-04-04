#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>

class PIDController {

private:

    // PID Tuning Parameters K_p, K_i, K_d
    // en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
    float m_k_p = 0.0f;
    float m_k_i = 0.0f;
    float m_k_d = 0.0f;

    // PID State 
    unsigned long m_last_microsecond_tick = 0.0f;
    float m_last_position_error = 0.0f;
    float m_accumulated_error = 0.0f;
    float m_setpoint_position = 0.0f;


public:
    PIDController(float k_p, float k_i, float k_d, float start_setpoint=0.0f);
    unsigned long tick();
    float plantOutput(float current_position);
    void changeSetpoint(float new_setpoint);
    void resetController();
};

#endif 
