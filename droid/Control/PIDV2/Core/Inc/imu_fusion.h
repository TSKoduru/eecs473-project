#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "math.h"
#include "Vectors.h"   // your existing Vector3 struct

typedef struct {
    float w, x, y, z;
} Quaternion;

void IMU_Fusion_Init(Quaternion *q);
void IMU_Update(Quaternion *q, struct Vector3 gyro, struct Vector3 accel,
                float dt, float tagYaw, int tagYawValid);
void Quaternion_ToEuler(Quaternion q, float *roll, float *pitch, float *yaw);
struct Vector3 Quaternion_Rotate(Quaternion q, struct Vector3 v);

#endif
