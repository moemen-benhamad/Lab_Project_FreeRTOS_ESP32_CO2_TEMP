#include <Arduino.h>
#include <DHTesp.h>
#include <Wire.h> 
#include <SSD1306Wire.h> 
#include <HardwareSerial.h>
#include "driver/uart.h"

#define PC_BUFFER_SIZE 10
#define N PC_BUFFER_SIZE
#define OLED_TASK_PERIOD 100
#define PRODUCER_TASK_PERIOD 1000
#define DHT22_TASK_PERIOD PRODUCER_TASK_PERIOD
#define CO2_TASK_PERIOD PRODUCER_TASK_PERIOD

#define DHT_PIN GPIO_NUM_15
#define DHT_TYPE DHT22
#define ESP_RX_PIN GPIO_NUM_26
#define ESP_TX_PIN GPIO_NUM_27
#define RX_BUF_SIZE 1024
#define ESP_UART_PORT UART_NUM_1 
#define SCREEN_I2C_ADDR 0x3C
#define SDA 5 
#define SCL 4 

DHTesp dht;
SSD1306Wire display(SCREEN_I2C_ADDR, SDA, SCL);

int i = 0; 
SemaphoreHandle_t s1 = NULL;
SemaphoreHandle_t s2 = NULL;
SemaphoreHandle_t mutex = NULL;
SemaphoreHandle_t s_sync_1 = NULL;
SemaphoreHandle_t s_sync_2 = NULL;

typedef enum {
    TEMPERATURE,
    HUMIDITY,
    CO2
} DataType;

typedef struct {
    DataType type; 
    float value;    
} SensorData;

SensorData buffer[PC_BUFFER_SIZE];

void vDht22_Task(void *pvParameters);
void vOled_Task(void *pvParameters);
void vCo2_Task(void *pvParameters);
void processSensorData(SensorData , float*, float*, int*);
void display_buffer(); // For testing