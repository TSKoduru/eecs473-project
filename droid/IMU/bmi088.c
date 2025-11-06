#include "Accel.h"
#include "Gyro.h"
#include "IMU.h"
#include "Vectors.h"

int main(void) {
    char msg[128];

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    IMU_INIT(&hspi1);
    IMU_SETUP_FOR_LOGGING();
    IMU_ENABLE_ALL();

    int imuready = IMU_READY();
    while(imuready != 0) {}

    struct Vector3 vec;

    while(1) {
        vec = ACCEL_READ_ACCELERATION();
	    vec = GYRO_READ_RATES();
    }
}