#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_16
#define TRIGGER_PIN GPIO_NUM_27
#define ECHO_PIN GPIO_NUM_26

#define RX_BUF_SIZE 1024
#define UART_PORT UART_NUM_1

uint8_t getCheckSum(uint8_t *packet);

void vmh_z19b_task(void *pvParameters){

    uint8_t data[9];
    uint8_t checksum;
    int value;

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_NUM_1, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    while(1){
        uart_write_bytes(UART_PORT, "\xFF\x01\x86\x00\x00\x00\x00\x00\x79", 9);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        if (uart_read_bytes(UART_PORT, data, 9, 1000 / portTICK_PERIOD_MS) == 9) {
            checksum = getCheckSum(data);
            if (data[0] == 0xff && data[1] == 0x86 && data[8] == checksum) {
                value = data[2] * 256 + data[3];
                printf("CO2 Concentration: %d ppm\n", value);
            } else {
                printf("Checksum error\n");
            }
        } else {
            printf("No response from MH-Z19B\n");
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

uint8_t getCheckSum(uint8_t *packet){
    int checksum = 0;
        for (int i = 1; i < 8; i++) {
        checksum += packet[i];
    }
    checksum = 0xff - checksum + 1;

    return checksum;
}

void vhcsr04_task(void *pvParameters)
{
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO_PIN, GPIO_PULLDOWN_ONLY);

    while (1) {
        // Send a 10us pulse to trigger pin
        gpio_set_level(TRIGGER_PIN, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        gpio_set_level(TRIGGER_PIN, 0);

        uint32_t start_time = 0;
        while (gpio_get_level(ECHO_PIN) == 0);
        start_time = esp_timer_get_time();

        uint32_t end_time = 0;
        while (gpio_get_level(ECHO_PIN) == 1);
        end_time = esp_timer_get_time();

        uint32_t pulse_duration = end_time - start_time;

        // Calculate distance in centimeters
        float distance_cm = (float)pulse_duration * 0.017; // Speed of sound : 343 m/s

        printf("Distance: %.2f cm", distance_cm);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


void app_main() {
    xTaskCreate(vmh_z19b_task, "mh_z19b_task", 4096, NULL, 5, NULL);
    xTaskCreate(vhcsr04_task, "hcsr04_task", 4096, NULL, 5, NULL);
    vTaskDelete(NULL);
}