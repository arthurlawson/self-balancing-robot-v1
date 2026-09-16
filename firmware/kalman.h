/**
 * @file kalman.h
 * @brief Kalman Filter Estimation System Header
 * @details Two-dimensional state estimation matrix pipeline designed for high-rate, IMU pitch and roll sensor fusion.
 * 
 * @author @author https://github.com/temperancee (Kalman filtering logic)
 */

#ifndef KALMAN_H
#define KALMAN_H

#define RAD_TO_DEG 57.2957795131f
#define SIGMA_INIT 0.1f    // Initial values on diagonal of sigma (i.e. initial roll and pitch variances)
#define R_INIT 0.001f       // Initial values on diagonal of R (process noise)
#define Q_INIT 0.05f       // Initial values on diagonal of Q (measurement noise)


class KalmanFilter {
    public:
        KalmanFilter(float Dt);
        void predict(float *gyro);
        void measurement_task(float *accel);
        float get_roll() const;
        float get_pitch() const;
        void reset();

    private:
        float roll, pitch;
        float Sigma[4];
        float R[2];
        float Q[2];
        float m_roll, m_pitch;
        float k_gain[4];
        float k_det;    
        float dt, dt_sec;
        bool is_initialized; // Refactor - added so that on startup the robot filter knows which angle immediately
};

#endif // KALMAN_H
