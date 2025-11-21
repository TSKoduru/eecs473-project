#ifndef EKF_H
#define EKF_H

/*
For final PID.c loop:

//before while(1)
EKF_Att ekf;
EKF_Init(&ekf);



// --- EKF ---

float gx = rpy_v.x;    // rad/s
float gy = rpy_v.y;
EKF_Predict(&ekf, gx, gy, dt);

// accel already in xyz_a
EKF_Update(&ekf, xyz_a.x, xyz_a.y, xyz_a.z);

// get EKF roll/pitch
rpy_p.x = EKF_Roll(&ekf);
rpy_p.y = EKF_Pitch(&ekf);


==== top code replaces bottom code =============
rpy_p.x += rpy_v.x * dt;
rpy_p.y += rpy_v.y * dt;

accel_roll  = atan2(y,z)
accel_pitch = ...
rpy_p.x = alpha*rpy_p.x + (1-alpha)*accel_roll
rpy_p.y = alpha*rpy_p.y + (1-alpha)*accel_pitch

*/


#include <stdint.h>

typedef struct {
    float x[4];     // [phi, theta, bgx, bgy]
    float P[4][4];  // Covariance
    float Q[4][4];  // Process noise
    float R[2][2];  // Measurement noise
} EKF;

// Initialize the filter
void EKF_Init(EKF *ekf);

// Predict step (gyro only)
void EKF_Predict(EKF *ekf, float gx, float gy, float dt);

// Update step (accel)
void EKF_Update(EKF *ekf, float ax, float ay, float az);

// retrives updated values:
static inline float EKF_Roll(EKF *ekf)  { return ekf->x[0]; }
static inline float EKF_Pitch(EKF *ekf) { return ekf->x[1]; }





#endif
