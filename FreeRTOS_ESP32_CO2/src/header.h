#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_16
#define ESP_RX_PIN GPIO_NUM_26
#define ESP_TX_PIN GPIO_NUM_27
#define ECHO_PIN GPIO_NUM_32
#define TRIGGER_PIN GPIO_NUM_33

#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024
#define UART_PORT UART_NUM_1
#define ESP_UART_PORT UART_NUM_2

void vmh_z19b_task(void *pvParameters);
void vhcsr04_task(void *pvParameters);
uint8_t getCheckSum(uint8_t*);