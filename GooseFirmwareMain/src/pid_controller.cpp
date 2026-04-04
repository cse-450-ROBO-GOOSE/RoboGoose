#include "pid_controller.hpp"

PIDController::PIDController(float k_p, float k_i, float k_d, float start_setpoint)
{
    this->m_k_p = k_p;
    this->m_k_i = k_i;
    this->m_k_d = k_d;
    this->m_setpoint_position = start_setpoint;
    this->m_last_microsecond_tick = this->tick();
}

unsigned long PIDController::tick()
{
    // this code is Giga specific due to needing a highly efficient micros() read
    // we use us_ticker_read() because this is much faster than calling micros()
    // since Arduino micros() on the Giga's mbed osforces atomic operations
    // to ensure multithreading consistency which we do not need for the PID loop
    // since we just need ticks that we can use for the piecewise integration
    // and multithreading time consistency **I believe** shouldn't matter
    // see: forum.arduino.cc/t/func-us-ticker-read-is-much-faster-than-micros-so/1288215/5
    return us_ticker_read();
}

float PIDController::plantOutput(float current_position){
    const float position_error = this->m_setpoint_position - current_position;

    const unsigned long current_tick = this->tick(); // if we get the control loop fine enough than could replace micros() with a constant
    const unsigned long ticks_elapsed = current_tick - m_last_microsecond_tick;

    const float velocity_error = (current_position - m_last_position_error) / ticks_elapsed; // division by zero should be safe since it should be impossible for ticks_elapsed==0 
    m_accumulated_error += (position_error * ticks_elapsed);
    m_last_position_error = position_error;


    const float proportional_term = m_k_p * position_error;
    const float integral_term = m_k_i * m_accumulated_error;
    const float derivative_term = m_k_d * velocity_error;

    const float plant_output = proportional_term + integral_term + derivative_term;
    return plant_output;
}

void PIDController::changeSetpoint(float new_setpoint){
    this->m_setpoint_position = new_setpoint;
}

void PIDController::resetController(){
    this->m_last_microsecond_tick = this->tick();
    this->m_last_position_error = 0.0f;
    this->m_accumulated_error = 0.0f;
}
