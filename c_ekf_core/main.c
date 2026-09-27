#include <stdio.h>
#include <math.h>
#include "mems_sensor_data.h"

// EKF State Variables
float state_angle = 0.0f;
float state_bias = 0.0f; // This will track and eliminate the physical gyro drift

// Covariance Matrix (P)
float P[2][2] = {
    {1.0f, 0.0f},
    {0.0f, 1.0f}
};

// Filter Tuning Parameters (Derived from physical noise characteristics)
#define Q_ANGLE 0.001f  // Process noise variance
#define Q_BIAS  0.003f  // Gyro bias variance
#define R_MEAS  3.0f    // Measurement noise variance (matches your MATLAB noise)

int main(void) {
    printf("--- Bare-Metal MEMS EKF Initialization ---\n");
    printf("Processing %d samples at %f sec/step...\n\n", DATA_LENGTH, DT);

    float filtered_angles[DATA_LENGTH];
    float rmse_error = 0.0f;

    for (int i = 0; i < DATA_LENGTH; i++) {
        // --- 1. PREDICT STEP (Using Gyroscope) ---
        // Physics model: New angle = Old angle + (Gyro Rate - Gyro Bias) * dt
        float rate = gyro_meas[i] - state_bias;
        state_angle += rate * DT;

        // Update Covariance Matrix (Jacobian propagation)
        P[0][0] += DT * (DT * P[1][1] - P[0][1] - P[1][0] + Q_ANGLE);
        P[0][1] -= DT * P[1][1];
        P[1][0] -= DT * P[1][1];
        P[1][1] += Q_BIAS * DT;

        // --- 2. UPDATE STEP (Using Accelerometer) ---
        // Calculate Innovation (Difference between Accel measurement and predicted angle)
        float y = accel_meas[i] - state_angle;

        // Calculate Innovation Covariance
        float S = P[0][0] + R_MEAS;

        // Calculate Kalman Gain (K)
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Update State with measurement
        state_angle += K[0] * y;
        state_bias += K[1] * y;

        // Update Covariance Matrix
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        // Store result and calculate error against the MATLAB ground truth
        filtered_angles[i] = state_angle;
        float error = true_angle[i] - state_angle;
        rmse_error += (error * error);
    }

    rmse_error = sqrtf(rmse_error / DATA_LENGTH);
    
    printf("--- EKF Execution Complete ---\n");
    printf("Final Estimated Gyro Bias: %f deg/s\n", state_bias);
    printf("Root Mean Square Error (RMSE): %f degrees\n", rmse_error);

    return 0;
}