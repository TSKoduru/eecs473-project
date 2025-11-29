#include "MadgwickAHRS.h" // adapted from https://github.com/arduino-libraries/MadgwickAHRS/tree/master/src

typedef struct {
    float px, py;         // position
    float vx, vy;         // velocity
    float yaw;            // heading
    float bax, bay;       // accel bias
} EKFState;

typedef struct {
    float P[7][7];        // covariance matrix
} EKFCov;

static EKFState ekf;
static EKFCov cov;

static const float Q_accel = 0.4f;     // process noise accel
static const float Q_bias  = 0.0001f;  // bias random walk
static const float R_pos   = 0.01f;    // AprilTag position noise
static const float R_yaw   = 0.05f;    // AprilTag yaw noise

MadgwickAHRS_t ahrs;

void EKF_Init(void)
{
    memset(&ekf, 0, sizeof(EKFState));
    memset(&cov, 0, sizeof(EKFCov));

    // Initialize covariance
    for(int i = 0; i < 7; i++)
        cov.P[i][i] = 0.1f;
}

void IMU_AHRS_Init(void)
{
    MadgwickAHRSInit(&ahrs, 200.0f); // update rate
}

// rotation into world frame
void rotate_accel_to_world(float ax, float ay, float az, float yaw, float *out_x, float *out_y)
{
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    *out_x = ax * cy - ay * sy;
    *out_y = ax * sy + ay * cy;
}

void EKF_Predict(float ax, float ay, float dt)
{
    // Remove accel bias
    ax -= ekf.bax;
    ay -= ekf.bay;

    // Convert body accel -> world accel
    float awx, awy;
    rotate_accel_to_world(ax, ay, 0.0f, ekf.yaw, &awx, &awy);

    // Integrate motion
    ekf.vx += awx * dt;
    ekf.vy += awy * dt;

    ekf.px += ekf.vx * dt;
    ekf.py += ekf.vy * dt;

    // Covariance prediction 
    float q = Q_accel * dt * dt;

    cov.P[0][0] += cov.P[2][2] * dt * dt + q;
    cov.P[1][1] += cov.P[3][3] * dt * dt + q;
    cov.P[2][2] += q;
    cov.P[3][3] += q;

    // Bias random walk
    cov.P[5][5] += Q_bias * dt;
    cov.P[6][6] += Q_bias * dt;
}

void EKF_Update_Position(float tag_x, float tag_y)
{
    float z_x = tag_x - ekf.px;
    float z_y = tag_y - ekf.py;

    float S = cov.P[0][0] + R_pos;
    float Kx = cov.P[0][0] / S;
    float Ky = cov.P[1][1] / S;

    ekf.px += Kx * z_x;
    ekf.py += Ky * z_y;

    cov.P[0][0] *= (1 - Kx);
    cov.P[1][1] *= (1 - Ky);
}

void EKF_Update_Yaw(float tag_yaw)
{
    float z = tag_yaw - ekf.yaw;

    // wrap to [-pi, pi]
    while(z >  M_PI) z -= 2*M_PI;
    while(z < -M_PI) z += 2*M_PI;

    float S = cov.P[4][4] + R_yaw;
    float K = cov.P[4][4] / S;

    ekf.yaw += K * z;
    cov.P[4][4] *= (1 - K);
}

uint32_t last = HAL_GetTick();

while(1)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - last) * 0.001f;
    last = now;

    // Read IMU 
    Vector3 acc = ACCEL_READ_ACCELERATION();
    Vector3 gyr = GYRO_READ_RATES();

    // Update AHRS 
    MadgwickAHRSupdate(&ahrs, gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z);

    ekf.yaw = MadgwickGetYaw(&ahrs);

    // EKF prediction 
    EKF_Predict(acc.x, acc.y, dt);

    // EKF correction (when tag packet arrives) 
    if (new_tag_data_available) {
        EKF_Update_Position(tag_x, tag_y);
        EKF_Update_Yaw(tag_yaw);
    }

    // use ekf.px, ekf.py in control loop
    control_loop(ekf.px, ekf.py, ekf.vx, ekf.vy, ekf.yaw);
}
