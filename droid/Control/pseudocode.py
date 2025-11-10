# constants, initializations
from cv2 import integral
from math import atan2, sqrt, sin, cos

def accel():
    # Placeholder function to read accelerometer data
    return (0.0, 0.0, 9.81)

def gyro():
    # Placeholder function to read gyroscope data
    return (0.0, 0.0, 0.0)

def matrix_mult(A, v):
    result = [0, 0, 0]
    for i in range(3):
        for j in range(3):
            result[i] += A[i][j] * v[j]
    return result

def R(phi, theta):
    R_x = [[1, 0, 0],
           [0, cos(phi), -sin(phi)],
           [0, sin(phi), cos(phi)]]
    
    R_y = [[cos(theta), 0, sin(theta)],
           [0, 1, 0],
           [-sin(theta), 0, cos(theta)]]
    
    return matrix_mult(R_y, R_x)

phi = 0.0    # roll
theta = 0.0  # pitch
dt = 0.01   # time step
alpha = 0.98  # complementary filter coefficient
Kp = 1.0    # proportional gain
Kd = 0.1    # derivative gain
Ki = 0.01   # integral gain
base = 1000  # base PWM value

# 1. Read sensors
a_x, a_y, a_z = accel()
gx, gy, gz = gyro()

# 2. Complementary filter for roll & pitch
phi_acc = atan2(a_y, a_z)
theta_acc = atan2(-a_x, sqrt(a_y*a_y + a_z*a_z))

phi_gyro = phi + gx * dt
theta_gyro = theta + gy * dt

phi   = alpha*phi_gyro   + (1-alpha)*phi_acc
theta = alpha*theta_gyro + (1-alpha)*theta_acc

# 3. Rotate accel into world frame
a_w = R(phi, theta) * [a_x, a_y, a_z]

# 4. Remove gravity
a_wz = a_w[2] - 9.81
a_wx = a_w[0]
a_wy = a_w[1]

# 5. Control law 
F_x = Kp*theta + Kd*gy + Ki*integral(a_wx, dt)
F_y = Kp*phi   + Kd*gx + Ki*integral(a_wy, dt)

# 6. Convert to motor commands
# prop configuration:
#     \*   */
#      \1 2/
#       \ /
#       / \
#      /3 4\
#     /*   *\
m1 = base - F_x + F_y
m2 = base + F_x + F_y
m3 = base - F_x - F_y
m4 = base + F_x - F_y