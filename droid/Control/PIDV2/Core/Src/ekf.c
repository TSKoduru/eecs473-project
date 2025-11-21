#include "ekf.h"
#include <math.h>


/**
 * Initializes the Extended Kalman Filter state, covariance,
 * process noise, and measurement noise.
 * sets up our x[4] matrix
 * 
 * estimates roll and pitch angles and gyro biases ( no yaw estimation )
 */
void EKF_Init(EKF *ekf)
{
    for (int i=0;i<4;i++) {
        for (int j=0;j<4;j++) {
            ekf->P[i][j] = 0; //covar
            ekf->Q[i][j] = 0; //processing noise
        }
    }

    //assumption that the droid is leveled before deployment
    ekf->x[0] = 0;  // roll
    ekf->x[1] = 0;  // pitch
    ekf->x[2] = 0;  // gyro bias x
    ekf->x[3] = 0;  // gyro bias y

    //covar p matrix init (values can be tuned)
    ekf->P[0][0] = 0.1f;   // roll uncertainty (~6°)
    ekf->P[1][1] = 0.1f;   // pitch uncertainty
    ekf->P[2][2] = 0.01f;  // small uncertainty in bias x
    ekf->P[3][3] = 0.01f;  // small uncertainty in bias y

    // Q matrix 
    // how much drifting is going on AKA large Q --> sensors more accurate than prediction
    ekf->Q[0][0] = 1e-5f;  // roll drift
    ekf->Q[1][1] = 1e-5f;  // pitch drift
    ekf->Q[2][2] = 1e-6f;  // gyro bias changes slowly
    ekf->Q[3][3] = 1e-6f;  // gyro bias y changes slowly

    ekf->R[0][0] = 0.03f; // ~degree noise
    ekf->R[1][1] = 0.03f;
}




/* Predicts the next state using gyroscope rates.
 * roll   += (gx - bias_x) * dt
 * pitch  += (gy - bias_y) * dt
 * bias_x = bias_x (bias drifts slowly)
 * bias_y = bias_y
 * updates the covariance matrix P using the Jacobian F.
 */

void EKF_Predict(EKF *ekf, float gx, float gy, float dt)
{

    // current state
    float phi   = ekf->x[0];
    float theta = ekf->x[1];
    float bgx   = ekf->x[2];
    float bgy   = ekf->x[3];

    // Gyro prediction: integrate angular velocity
    ekf->x[0] = phi   + (gx - bgx) * dt;   // predicted roll
    ekf->x[1] = theta + (gy - bgy) * dt;   // predicted pitch

    /* Jacobian matrix f:
     * Partial derivatives of the state transition function.
     * This models how small changes propagate through time.
     */
    float F[4][4] = {
        {1, 0, -dt,  0},   // roll depends on bias_x
        {0, 1,  0, -dt},   // pitch depends on bias_y
        {0, 0,  1,  0},    // bias_x stays the same
        {0, 0,  0,  1}     // bias_y stays the same
    };

    float Pnew[4][4] = {0};


    // P = F*P*F^T + Q
    for (int i=0;i<4;i++){
        for (int j=0;j<4;j++){
            float sum = 0;
            for (int k=0;k<4;k++){
                for (int l=0;l<4;l++){
                    sum += F[i][k] * ekf->P[k][l] * F[j][l];
                }
            }
            Pnew[i][j] = sum + ekf->Q[i][j];
        }
    }

    for (int i=0;i<4;i++)
        for (int j=0;j<4;j++)
            ekf->P[i][j] = Pnew[i][j];
}




/* corrects prediction using accelerometer.
 * z = [accel_roll, accel_pitch]
 * updates covariance, kalman gain value (k), corrected state, and measurement residual
 */
void EKF_Update(EKF *ekf, float ax, float ay, float az)
{
    //accel data --> roll and pitch angles (same as compl. filter)
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    // measurement (z)
    float z[2] = { roll, pitch };

    // predicted measurement (just roll & pitch from the state)
    float h[2] = { ekf->x[0], ekf->x[1] };

    // innovation y = z - h(x)
    float y[2] = { z[0]-h[0], z[1]-h[1] };

    /* Innovation covariance S = H*P*H^T + R
     * smth about Jacobian need to check back later...
     */
    float S[2][2] = {
        { ekf->P[0][0] + ekf->R[0][0], ekf->P[0][1] },
        { ekf->P[1][0], ekf->P[1][1] + ekf->R[1][1] }
    };

    // determinant check to avoid division by zero
    float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (fabs(det) < 1e-9f) return;

    float invDet = 1.0f / det;

    // invert S (2x2 matrix)
    float Sinv[2][2] = {
        {  S[1][1]*invDet, -S[0][1]*invDet },
        { -S[1][0]*invDet,  S[0][0]*invDet }
    };

    // Kalman Gain K = P * H^T * S^-1
    float K[4][2] = {0};

    for (int i=0;i<4;i++){
        K[i][0] = ekf->P[i][0]*Sinv[0][0] + ekf->P[i][1]*Sinv[1][0];
        K[i][1] = ekf->P[i][0]*Sinv[0][1] + ekf->P[i][1]*Sinv[1][1];
    }

    //Correct the state x = x + K*y

    for (int i=0;i<4;i++)
        ekf->x[i] += K[i][0]*y[0] + K[i][1]*y[1];

    // update covariance (P = (I - K*H)*P)
    float I_KH[4][4] = {
        {1-K[0][0], -K[0][1], 0, 0},
        { -K[1][0], 1-K[1][1], 0, 0},
        { -K[2][0], -K[2][1], 1, 0},
        { -K[3][0], -K[3][1], 0, 1}
    };

    float Pnew[4][4] = {0};
    for (int i=0;i<4;i++){
        for (int j=0;j<4;j++){
            for (int k=0;k<4;k++){
                Pnew[i][j] += I_KH[i][k] * ekf->P[k][j];
            }
        }
    }

    // writes back updated covariance
    for (int i=0;i<4;i++)
        for (int j=0;j<4;j++)
            ekf->P[i][j] = Pnew[i][j];
}
