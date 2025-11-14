/*
 * imu_fusion.c
 *
 *  Created on: Nov 13, 2025
 *      Author: Cindy
 */


#include "imu_fusion.h"

static void normalizeQ(Quaternion *q) {
    float n = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (n < 1e-6f) return;
    q->w /= n; q->x /= n; q->y /= n; q->z /= n;
}

static Quaternion qMul(Quaternion a, Quaternion b) {
    Quaternion r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

struct Vector3 Quaternion_Rotate(Quaternion q, struct Vector3 v) {
    Quaternion vq = {0, v.x, v.y, v.z};
    Quaternion q_conj = {q.w, -q.x, -q.y, -q.z};
    Quaternion tmp = qMul(q, vq);
    Quaternion res = qMul(tmp, q_conj);
    struct Vector3 r = {res.x, res.y, res.z};
    return r;
}

void IMU_Fusion_Init(Quaternion *q) {
    q->w = 1.0f;
    q->x = q->y = q->z = 0.0f;
}

void IMU_Update(Quaternion *q, struct Vector3 gyro, struct Vector3 accel,
                float dt, float tagYaw, int tagYawValid)
{
    // ---- Gyro integration ----
    float gx = gyro.x, gy = gyro.y, gz = gyro.z;
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    Quaternion dq;
    dq.w = 1.0f;
    dq.x = gx; dq.y = gy; dq.z = gz;

    *q = qMul(*q, dq);
    normalizeQ(q);

    // ---- Accelerometer correction ----
    float ax = accel.x, ay = accel.y, az = accel.z;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 1e-3f) {
        ax /= norm; ay /= norm; az /= norm;

        // Predicted gravity vector from quaternion
        float gx_pred = 2.0f * (q->x*q->z - q->w*q->y);
        float gy_pred = 2.0f * (q->w*q->x + q->y*q->z);
        float gz_pred = q->w*q->w - q->x*q->x - q->y*q->y + q->z*q->z;

        // Error between measured and predicted gravity
        struct Vector3 err = {
            (ay*gz_pred - az*gy_pred),
            (az*gx_pred - ax*gz_pred),
            (ax*gy_pred - ay*gx_pred)
        };

        float k_acc = 0.05f; // tuning gain
        q->w += (-q->x*err.x - q->y*err.y - q->z*err.z) * k_acc * dt;
        q->x += ( q->w*err.x + q->y*err.z - q->z*err.y) * k_acc * dt;
        q->y += ( q->w*err.y - q->x*err.z + q->z*err.x) * k_acc * dt;
        q->z += ( q->w*err.z + q->x*err.y - q->y*err.x) * k_acc * dt;
        normalizeQ(q);
    }

    // ---- Optional: yaw correction from AprilTag ----
    if (tagYawValid) {
        // Extract current yaw from quaternion
        float siny_cosp = 2.0f * (q->w*q->z + q->x*q->y);
        float cosy_cosp = 1.0f - 2.0f * (q->y*q->y + q->z*q->z);
        float imuYaw = atan2f(siny_cosp, cosy_cosp);

        float yaw_err = tagYaw - imuYaw;
        while (yaw_err > M_PI) yaw_err -= 2*M_PI;
        while (yaw_err < -M_PI) yaw_err += 2*M_PI;

        float k_yaw = 0.1f; // yaw fusion gain
        float half_yaw_err = 0.5f * yaw_err * k_yaw;
        Quaternion q_corr = {cosf(half_yaw_err), 0, 0, sinf(half_yaw_err)};
        *q = qMul(q_corr, *q);
        normalizeQ(q);
    }
}

void Quaternion_ToEuler(Quaternion q, float *roll, float *pitch, float *yaw)
{
    *roll = atan2f(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y));
    *pitch = asinf(2*(q.w*q.y - q.z*q.x));
    *yaw = atan2f(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
}
