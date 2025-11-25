#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

typedef struct {
    float beta;
    float q0, q1, q2, q3;
    float sampleFreq;
} MadgwickAHRS_t;

void MadgwickAHRSInit(MadgwickAHRS_t *ahrs, float sampleFreq);
void MadgwickAHRSupdate(MadgwickAHRS_t *ahrs, float gx, float gy, float gz,
                        float ax, float ay, float az);

// Convenience functions
float MadgwickGetRoll(MadgwickAHRS_t *ahrs);
float MadgwickGetPitch(MadgwickAHRS_t *ahrs);
float MadgwickGetYaw(MadgwickAHRS_t *ahrs);

#endif
