#include <math.h>
#include "MadgwickAHRS.h"

// Initialize filter
void MadgwickAHRSInit(MadgwickAHRS_t *ahrs, float sampleFreq)
{
    ahrs->beta = 0.1f;
    ahrs->sampleFreq = sampleFreq;

    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
}

// Madgwick filter update
void MadgwickAHRSupdate(MadgwickAHRS_t *ahrs, float gx, float gy, float gz,
                        float ax, float ay, float az)
{
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
    float beta = ahrs->beta;
    float recipNorm;

    // Normalise accelerometer measurement
    recipNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (recipNorm == 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Gradient descent algorithm corrective step
    float f1 = 2*(q1*q3 - q0*q2) - ax;
    float f2 = 2*(q0*q1 + q2*q3) - ay;
    float f3 = 2*(0.5f - q1*q1 - q2*q2) - az;

    float J_11or24 = 2*q2;
    float J_12or23 = 2*q3;
    float J_13or22 = 2*q0;
    float J_14or21 = 2*q1;
    float J_32 = 2*J_14or21;
    float J_33 = 2*J_11or24;

    float step0 = J_14or21*f2 - J_11or24*f1;
    float step1 = J_12or23*f1 + J_13or22*f2 - J_32*f3;
    float step2 = J_12or23*f2 - J_33*f3 - J_13or22*f1;
    float step3 = J_14or21*f1 + J_11or24*f2;

    // Normalise step
    recipNorm = sqrtf(step0*step0 + step1*step1 + step2*step2 + step3*step3);
    recipNorm = 1.0f / recipNorm;
    step0 *= recipNorm;
    step1 *= recipNorm;
    step2 *= recipNorm;
    step3 *= recipNorm;

    // Apply feedback step
    float gx_rad = gx;
    float gy_rad = gy;
    float gz_rad = gz;

    gx_rad -= beta * step0;
    gy_rad -= beta * step1;
    gz_rad -= beta * step2;

    // Integrate rate of change of quaternion
    gx_rad *= (0.5f / ahrs->sampleFreq);
    gy_rad *= (0.5f / ahrs->sampleFreq);
    gz_rad *= (0.5f / ahrs->sampleFreq);

    q0 += (-q1*gx_rad - q2*gy_rad - q3*gz_rad);
    q1 += ( q0*gx_rad + q2*gz_rad - q3*gy_rad);
    q2 += ( q0*gy_rad - q1*gz_rad + q3*gx_rad);
    q3 += ( q0*gz_rad + q1*gy_rad - q2*gx_rad);

    // Normalise quaternion
    recipNorm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    recipNorm = 1.0f / recipNorm;
    ahrs->q0 = q0 * recipNorm;
    ahrs->q1 = q1 * recipNorm;
    ahrs->q2 = q2 * recipNorm;
    ahrs->q3 = q3 * recipNorm;
}

// ------------ Euler angle helpers -----------------

float MadgwickGetRoll(MadgwickAHRS_t *ahrs)
{
    return atan2f(2.0f*(ahrs->q0*ahrs->q1 + ahrs->q2*ahrs->q3),
                  1 - 2*(ahrs->q1*ahrs->q1 + ahrs->q2*ahrs->q2));
}

float MadgwickGetPitch(MadgwickAHRS_t *ahrs)
{
    return asinf(2.0f*(ahrs->q0*ahrs->q2 - ahrs->q3*ahrs->q1));
}

float MadgwickGetYaw(MadgwickAHRS_t *ahrs)
{
    return atan2f(2.0f*(ahrs->q0*ahrs->q3 + ahrs->q1*ahrs->q2),
                  1 - 2*(ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3));
}
