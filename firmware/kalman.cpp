#include "kalman.h"
#include <math.h>

float sqr(float x) { return x * x; }

KalmanFilter::KalmanFilter(float Dt) {
    dt = Dt;
    reset();
}

void KalmanFilter::predict(float *gyro) {
    dt_sec = dt / 1000.0f;
    
    // Predict step
    roll += gyro[0] * RAD_TO_DEG * dt_sec; // convert rad to degrees
    pitch += gyro[1] * RAD_TO_DEG * dt_sec;

    Sigma[0] += R[0] * dt_sec; // added dt
    Sigma[3] += R[1] * dt_sec; // added dt
}


void KalmanFilter::measurement_task(float *accel) {
    m_roll = atan2( accel[1], sqrt( sqr(accel[0]) + sqr(accel[2]) ) ) * RAD_TO_DEG;
    m_pitch = atan2( -accel[0], sqrt( sqr(accel[1]) + sqr(accel[2]) ) ) * RAD_TO_DEG;

    if (!is_initialized) {
        roll = m_roll;
        pitch = m_pitch;
        is_initialized = true;
        return;
    }
    
    // Kalman Gain
    float S0 = Sigma[0] + Q[0];
    float S1 = Sigma[1];
    float S2 = Sigma[2];
    float S3 = Sigma[3] + Q[1];

    k_det = 1.0f / (S0 * S3 - S1 * S2);

    k_gain[0] = (Sigma[0] * S3 - Sigma[1] * S2) * k_det;
    k_gain[1] = (Sigma[1] * S0 - Sigma[0] * S1) * k_det;
    k_gain[2] = (Sigma[2] * S3 - Sigma[3] * S2) * k_det;
    k_gain[3] = (Sigma[3] * S0 - Sigma[2] * S1) * k_det;

    // Update State
    float r_error = m_roll - roll;
    float p_error = m_pitch - pitch;
    roll += (k_gain[0] * r_error) + (k_gain[1] * p_error);
    pitch += (k_gain[2] * r_error) + (k_gain[3] * p_error);

    float s0 = Sigma[0], s1 = Sigma[1], s2 = Sigma[2], s3 = Sigma[3];

    Sigma[0] = (1.0f - k_gain[0]) * s0 - k_gain[1] * s2;
    Sigma[1] = (1.0f - k_gain[0]) * s1 - k_gain[1] * s3;
    Sigma[2] = -k_gain[2] * s0 + (1.0f - k_gain[3]) * s2;
    Sigma[3] = -k_gain[2] * s1 + (1.0f - k_gain[3]) * s3;

    float relationship_avg = (Sigma[1] + Sigma[2]) * 0.5f;
    Sigma[1] = Sigma[2] = relationship_avg;
}

float KalmanFilter::get_roll() const { return roll; }
float KalmanFilter::get_pitch() const { return pitch; }

void KalmanFilter::reset() {
    is_initialized = false;
    roll = 0.0f;
    pitch = 0.0f;
    Sigma[0] = Sigma[3] = SIGMA_INIT;
    Sigma[1] = Sigma[2] = 0.0f;
    R[0] = R[1] = R_INIT;
    Q[0] = Q[1] = Q_INIT;
}
