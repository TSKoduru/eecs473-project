STM32 port:https://github.com/ParkerHitch/STM32-BMI088


Modify lines 9 & 10 of Accel.h and Gyro.h to refelt your GPIO chip select ports/pins.
// Set port and pin for accelerometer here
#define ACCEL_CS_PORT YOUR_PORT_HERE
#define ACCEL_CS_PIN YOUR_PIN_HERE
Pass your SPI handler struct into IMU_INIT, if you are using both sensors, or into ACCEL/GYRO_INIT if you are just using one.

Call IMU_ENABLE_ALL or respective gyro/accel enabling functions (see internals of IMU_ENABLE_ALL in IMU.c).

(Optional) perform self-tests.

Enjoy!