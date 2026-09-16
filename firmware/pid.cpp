#include "Arduino.h"
#include "pid.h"
#include <math.h>

PID::PID(float Setpoint, float Kp, float Ki, float Kd, float Dt)
{
    setpoint = Setpoint;
    kp = Kp;
    ki = Ki;
    kd = Kd;
    dt = Dt;
    reset();
}

// max is the absolute (i.e. positive) maximum - used for the anti-windup clamping
float PID::control(float pv, float max)
{
    error = setpoint - pv;

    // ========== DERIVATIVE ===========
    float raw_derivative = (error - previous_error) / dt;
    derivative = (0.80f * previous_derivative) + (0.2f * raw_derivative); // low pass filter
    
    previous_error = error;
    previous_derivative = derivative;

    // ========== INTEGRAL ==========
    integral = constrain((integral + (error * dt)), -1.20f, 1.20f);

    // ========== OUTPUT ==========
    out = (kp * error) + (ki * integral) + (kd * derivative);
    return constrain(out, -max, max); 

}

void PID::set_setpoint(float Setpoint) {
    setpoint = Setpoint;
}

void PID::set_kp(float Kp) {
    kp = Kp;
}

void PID::set_kd(float Kd) {
    kd = Kd;
}

float PID::get_derivative() const { return derivative; }

void PID::leak_integral(float percentage) { integral *= percentage; }

void PID::reset() {
    integral = 0.0f;
    derivative = 0.0f;
    error = 0.0f;
    previous_error = 0.0f;
    previous_derivative = 0.0f;
    out = 0.0f;
}
