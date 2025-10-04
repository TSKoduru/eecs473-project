#include "main.h"
#include <string.h>

UART_HandleTypeDef huart2;

uint8_t rx_buffer[32];  // Incoming message buffer
uint8_t rx_byte;
uint8_t rx_index = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rx_byte == '\n' || rx_index >= sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index] = '\0';  // Null-terminate
            rx_index = 0;

            if (strcmp((char*)rx_buffer, "HELLO") == 0) {
                char msg[] = "Received HELLO\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
            } else if (strcmp((char*)rx_buffer, "STATUS") == 0) {
                char msg[] = "Received STATUS\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            } else {
                char msg[] = "Unknown command\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
        } else {
            rx_buffer[rx_index++] = rx_byte;
        }

        // Continue receiving
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // Start UART in interrupt mode
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

    while (1) {
        // Main loop can remain empty or do other tasks
    }
}
