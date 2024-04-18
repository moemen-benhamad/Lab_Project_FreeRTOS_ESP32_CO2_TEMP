#define PC_BUFFER_SIZE 10
#define N PC_BUFFER_SIZE
int i = 0; 
#define DISPLAY_TIMEOUT 10000


/*---------------------- COMPILER FLAGS -----------------------*/
//#define SHOW_ERROR_MSGS
//#define SHOW_IMPORTANT_ERROR_MSGS
//#define SHOW_SENSOR_MEASURMENTS
/*-------------------------------------------------------------*/


/*---------------------- TASK PERIODS -------------------------*/
#define UART_RX_TASK_PERIOD 500
#define PRODUCER_TASK_PERIOD 4000
#define CONSUMER_TASK_PERIOD 1000
#define OLED_TASK_PERIOD CONSUMER_TASK_PERIOD
#define DHT22_TASK_PERIOD PRODUCER_TASK_PERIOD
#define CO2_TASK_PERIOD PRODUCER_TASK_PERIOD
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


/*------------------- WiFi & Server Config  --------------------*/
const char* ssid = "ESP32-AP";
const char* password = "password";
const char* hostname = "esp32";
WebServer server(80);
float server_temperature = 0;
float server_humidity = 0;
float server_co2 = 0;
/*-------------------------------------------------------------*/


/*-------------------- SEMAPHORES & MUTEXS --------------------*/
SemaphoreHandle_t s1 = NULL;
SemaphoreHandle_t s2 = NULL;
SemaphoreHandle_t mutex = NULL;
SemaphoreHandle_t s_wake_display = NULL;
/*-------------------------------------------------------------*/


/*---------------------- VARS & TYPEDEFS ----------------------*/
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

SensorData buffer[PC_BUFFER_SIZE];
/*-------------------------------------------------------------*/

TaskHandle_t voled_task_handle;

/*---------------------- FUNCTION HEADERS ----------------------*/

void vdht22_Task(void *pvParameters);
void voled_Task(void *pvParameters);
void vuart_rx_task(void *pvParameters);
void vweb_server_Task(void *pvParameters);
void processSensorData(SensorData , float*, float*, int*);
void handleRoot();

/*-------------------------------------------------------------*/
