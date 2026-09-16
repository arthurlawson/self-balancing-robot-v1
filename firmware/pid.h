/**
 * @file pid.h
 * @brief Proportional-Integral-Derivative (PID) Controller Module
 * @details Implements a real-time feedback loop controller featuring 
 *          with active anti-windup integral leakage mitigation pipeline.
 * 
 * @author https://github.com/temperancee (PID system)
 */

#ifndef PID_H
#define PID_H

class PID {
    public:
        PID(float Setpoint, float Kp, float Ki, float Kd, float Dt);
        float control(float pv, float max);
        void set_setpoint(float Setpoint);
        void set_kp(float Kp);
        void set_kd(float Kd); 
        float get_derivative() const;
        void leak_integral(float percentage);
        void reset();

    private:
        float kp, ki, kd, dt;
        float setpoint;
        float integral;
        float previous_error;
        float previous_derivative;
        
        // Pre allocated to save memory
        float error;
        float derivative;
        float out;
};

#endif // !PID_H
