/*  ------------------------------------------------- WARNING --------------------------------------------------------------

  !CONNECT RED WIRE TO 5V WHEN POWERING WITH USB.
  !CONNECT RED WIRE TO VCC WHEN POWERING WITH AN EXERNAL POWER SOURCE (ESP32), other with the oled display won't turn on.

-------------------------------------------------------------------------------------------------------------------------- */


/*---------------------- COMPILER FLAGS -----------------------*/

/*                  :: Uncomment to use ::                     */

// #define DISPLAY_ALWAYS_ON
 #define DISPLAY_STATE_MSGS
#define ESP_UART_MSGS
#define SHOW_DHT22_MEASURMENTS
// #define SHOW_DHT22_ERROR_MSGS

/*-------------------------------------------------------------*/


/*--------------------- DISPLAY_TIMEOUT -----------------------*/
#define DISPLAY_TIMEOUT 10000
/*-------------------------------------------------------------*/


/*---------------------- TASK PERIODS -------------------------*/
#define UART_RX_TASK_PERIOD 100
#define WEBSERVER_TASK_PERIOD 100
#define CONSUMER_TASK_PERIOD 1000
#define PRODUCER_TASK_PERIOD 4000
#define OLED_TASK_PERIOD CONSUMER_TASK_PERIOD
#define DHT22_TASK_PERIOD PRODUCER_TASK_PERIOD
#define CO2_TASK_PERIOD PRODUCER_TASK_PERIOD
/*-------------------------------------------------------------*/


/*------------------- WiFi & Server Config  --------------------*/
const char* ssid = "ESP32-AP";
const char* password = "password";
const char* hostname = "esp32";
WebServer server(80);
float server_temperature = 0;
float server_humidity = 0;
float server_co2 = 0;
/*-------------------------------------------------------------*/


/*------------------------ DHT22 CONFIG ------------------------*/
#define DHT_PIN GPIO_NUM_15
#define DHT_TYPE DHT22
DHTesp dht;
/*-------------------------------------------------------------*/


/*------------------------ UART CONFIG ------------------------*/
#define RX_BUF_SIZE 1024
#define ESP_RX_PIN GPIO_NUM_26
#define ESP_TX_PIN GPIO_NUM_27
#define ESP_UART_PORT UART_NUM_1
/*-------------------------------------------------------------*/


/*------------------------ I2C CONFIG -------------------------*/
#define SCREEN_I2C_ADDR 0x3C
#define SDA 5 
#define SCL 4
SSD1306Wire display(SCREEN_I2C_ADDR, SDA, SCL);
/*-------------------------------------------------------------*/


/*-------------------- SEMAPHORES & MUTEXS --------------------*/
#define N 2
int i = 0; 
SemaphoreHandle_t s1 = NULL;
SemaphoreHandle_t s2 = NULL;
SemaphoreHandle_t mutex = NULL;
SemaphoreHandle_t s_wake_display = NULL;
/*-------------------------------------------------------------*/


/*---------------------- VARS & TYPEDEFS ----------------------*/
uint8_t display_isOn = false;

typedef enum {
    TEMPERATURE,
    HUMIDITY,
    CO2
} DataType;

typedef struct {
    DataType type; 
    float value;    
} SensorData;

typedef struct {
    int co2_value;
    uint8_t wake_diplay_signal; 
} UartSerialData;

UartSerialData uart_serial_data;

SensorData buffer[N];
/*-------------------------------------------------------------*/


/*---------------------- FUNCTION HEADERS ----------------------*/

void vdht22_Task(void *pvParameters);
void voled_Task(void *pvParameters);
void vuart_rx_task(void *pvParameters);
void vweb_server_Task(void *pvParameters);
void processSensorData(SensorData , float*, float*, int*);
void handleRoot();

/*-------------------------------------------------------------*/
