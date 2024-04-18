/*---------------------- COMPILER FLAGS -----------------------*/

/*                  :: Uncomment to use ::                     */

#define USE_PWM_FOR_CO2 // Comment this line to use uart instead
#define SHOW_MH_Z19B_MEASURMENTS
// #define SHOW_HCR04_MEASURMENTS
#define SHOW_UART_MSGS
// #define SHOW_MH_Z19B_ERROR_MSGS

/*-------------------------------------------------------------*/


/*---------------------- TASK PERIODS -------------------------*/
#define MH_Z19B_TASK_DELAY 4000
#define HCR04_TASK_DELAY 100
/*-------------------------------------------------------------*/


/*------------------------ GPIO CONFIG ------------------------*/
#define ECHO_PIN GPIO_NUM_32
#define TRIGGER_PIN GPIO_NUM_33
/*-------------------------------------------------------------*/


/*------------------------ PWM CONFIG -------------------------*/
#define PWM_PIN GPIO_NUM_23
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_FREQ_HZ 1000
#define PWM_RESOLUTION LEDC_TIMER_13_BIT
/*-------------------------------------------------------------*/


/*------------------------ UART CONFIG ------------------------*/
#define RX_BUF_SIZE 1024
#define WAKE_UP_DISTANCE_CM 8

// MH-Z19B
#define TX_PIN GPIO_NUM_16
#define RX_PIN GPIO_NUM_17
#define UART_PORT UART_NUM_1

// ESP32_OLED
#define ESP_RX_PIN GPIO_NUM_26
#define ESP_TX_PIN GPIO_NUM_27
#define ESP_UART_PORT UART_NUM_2
/*-------------------------------------------------------------*/


/*-------------------- SEMAPHORES & MUTEXS --------------------*/
SemaphoreHandle_t mutex = NULL;
SemaphoreHandle_t s_sync = NULL;
SemaphoreHandle_t uart_sync = NULL;
/*-------------------------------------------------------------*/


/*---------------------- VARS & TYPEDEFS ----------------------*/

typedef struct {
    int co2_value;
    uint8_t wake_diplay_signal;     
} UartSerialData;

UartSerialData uart_serial_data;

uint64_t lastNegEdgeTime = 0, lastPosEdgeTime = 0;
uint64_t tH = 0, tL = 0;
uint64_t Cppm = 0;
/*-------------------------------------------------------------*/


/*---------------------- FUNCTION HEADERS ----------------------*/

void vmh_z19b_uart_task(void *pvParameters);
void vmh_z19b_pwm_task(void *pvParameters);
void vhcsr04_task(void *pvParameters);
void vuart_tx_task(void *pvParameters);
void sspi(void *arg);
uint8_t getCheckSum(uint8_t*);

/*-------------------------------------------------------------*/