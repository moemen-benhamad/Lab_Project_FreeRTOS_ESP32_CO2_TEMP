#include <Arduino.h>
#include <DHTesp.h>
#include <Wire.h> 
#include <SSD1306Wire.h> 
#include <HardwareSerial.h>
#include "driver/uart.h"

#define DHT_PIN GPIO_NUM_15
#define DHT_TYPE DHT22
#define ESP_RX_PIN GPIO_NUM_26
#define ESP_TX_PIN GPIO_NUM_27
#define RX_BUF_SIZE 1024
#define ESP_UART_PORT UART_NUM_1
#define OLED_ADDR 0x3C
#define SDA 5 
#define SCL 4  

#define SCREEN_WIDTH 128 // !!!!!!!
#define SCREEN_HEIGHT 64 // !!!!!!!

DHTesp dht;
SSD1306Wire display(OLED_ADDR, SDA, SCL);
 
int N = 10;
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

SensorData buffer[200];

void vDht22_Task(void *pvParameters);
void vOled_Task(void *pvParameters);
void vco2_Task(void *pvParameters);
void processSensorData(SensorData , float*, float*, int*);
