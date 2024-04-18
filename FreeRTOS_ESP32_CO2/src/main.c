#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "header.h"

void app_main() {
    s_sync = xSemaphoreCreateBinary();
    mutex = xSemaphoreCreateMutex();
    uart_sync = xSemaphoreCreateBinary();

    gpio_reset_pin(PWM_PIN);
    gpio_set_direction(PWM_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(PWM_PIN, GPIO_INTR_ANYEDGE);
  
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PWM_PIN, sspi, (void*) PWM_PIN);

    #ifdef USE_PWM_FOR_CO2
        xTaskCreatePinnedToCore(vmh_z19b_pwm_task, "mh_z19b_task", 4096, NULL, 2, NULL,0);
    #else
        xTaskCreatePinnedToCore(vmh_z19b_uart_task, "mh_z19b_task", 4096, NULL, 1, NULL,0);
    #endif
    xTaskCreatePinnedToCore(vhcsr04_task, "hcsr04_task", 4096, NULL, 1, NULL,0);
    xTaskCreatePinnedToCore(vuart_tx_task, "uart_tx_task", 4096, NULL, 1, NULL,0);

    vTaskDelete(NULL);
}

void vmh_z19b_uart_task(void *pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t data[9];
    uint8_t checksum;
    int co2_value;

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
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    if(uart_write_bytes(UART_PORT, "\xFF\x01\x99\x00\x00\x00\x13\x88\xCB", 9) == 9){
        #ifdef SHOW_ERROR_MSGS
            printf("Range changed to 5000 successfully.\n");
        #endif
    }
    else {
        #ifdef SHOW_ERROR_MSGS
            printf("Range checksum error!\n");
        #endif
    };

    for(;;){
        uart_write_bytes(UART_PORT, "\xFF\x01\x86\x00\x00\x00\x00\x00\x79", 9);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MH_Z19B_TASK_DELAY)); // OK?

        if (uart_read_bytes(UART_PORT, data, 9, portMAX_DELAY) == 9) {
            checksum = getCheckSum(data);
            if (data[0] == 0xff && data[1] == 0x86 && data[8] == checksum) {
                co2_value = data[2] * 256 + data[3];
                if(co2_value > 0){
                    xSemaphoreTake(mutex, portMAX_DELAY);
                    uart_serial_data.co2_value = co2_value;
                    xSemaphoreGive(mutex);
                    xSemaphoreGive(uart_sync);
                }
                
                #ifdef SHOW_SENSOR_MEASURMENTS
                    printf("CO2 Concentration [UART]: %d ppm\n", co2_value);
                #endif

            } else {
                #ifdef SHOW_ERROR_MSGS
                    printf("Checksum error!\n");
                #endif
            }
        } else {
            #ifdef SHOW_ERROR_MSGS
                printf("No response from MH-Z19B!\n");
            #endif
        }
    }
    vTaskDelete(NULL);
}

void vhcsr04_task(void *pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO_PIN, GPIO_PULLDOWN_ONLY);
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    uint32_t pulse_duration = 0;
    float distance_cm = 0;

    for(;;) {
        gpio_set_level(TRIGGER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(TRIGGER_PIN, 0);

        start_time = 0;
        while (gpio_get_level(ECHO_PIN) == 0);
        start_time = esp_timer_get_time();

        end_time = 0;
        while (gpio_get_level(ECHO_PIN) == 1);
        end_time = esp_timer_get_time();

        pulse_duration = end_time - start_time;
        distance_cm = (float)pulse_duration * 0.017; // Speed of sound : 343 m/s
        if(distance_cm <= WAKE_UP_DISTANCE_CM){
            xSemaphoreTake(mutex, portMAX_DELAY);
            uart_serial_data.wake_diplay_signal = 1;
            xSemaphoreGive(mutex);
            xSemaphoreGive(uart_sync);
        }

        #ifdef SHOW_SENSOR_MEASURMENTS
            printf("Distance: %.2f cm\n", distance_cm);
        #endif

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HCR04_TASK_DELAY));
    }
    vTaskDelete(NULL);
}

void vuart_tx_task(void *pvParameters){

    const uart_config_t uart_config_esp = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(ESP_UART_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(ESP_UART_PORT, &uart_config_esp);
    uart_set_pin(ESP_UART_PORT, ESP_TX_PIN, ESP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    UartSerialData uart_serial_data_local;

    for(;;) {
        xSemaphoreTake(uart_sync, portMAX_DELAY);
        xSemaphoreTake(mutex, portMAX_DELAY);
            uart_serial_data_local = uart_serial_data;
        xSemaphoreGive(mutex);
        if(uart_write_bytes(ESP_UART_PORT, &uart_serial_data_local, 5) == 5){
            #ifdef SHOW_IMPORTANT_ERROR_MSGS
                printf("UART data successfully sent to ESP32 OLED: %d|%d\n", uart_serial_data_local.wake_diplay_signal, uart_serial_data_local.co2_value);
            #endif
        }
        else {
            #ifdef SHOW_IMPORTANT_ERROR_MSGS
                printf("Error sending UART data reading to ESP32 OLED\n");
            #endif
        }
        if(uart_serial_data_local.wake_diplay_signal == 1){
            xSemaphoreTake(mutex, portMAX_DELAY);
            uart_serial_data.wake_diplay_signal = 0;
            xSemaphoreGive(mutex);
        }
    }
    vTaskDelete(NULL);
}

void vmh_z19b_pwm_task( void *pvParameters ){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;){
        if(Cppm > 0){
            xSemaphoreTake(s_sync, portMAX_DELAY);
            xSemaphoreTake(mutex, portMAX_DELAY);
            uart_serial_data.co2_value = (int) Cppm;
            xSemaphoreGive(mutex);
            xSemaphoreGive(uart_sync);
        }
        #ifdef SHOW_SENSOR_MEASURMENTS
            printf("CO2 concentration [PWM]: %llu ppm\n", Cppm);
        #endif

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MH_Z19B_TASK_DELAY));
    }
    vTaskDelete(NULL);
}

void sspi(void *arg){
    uint64_t now = esp_timer_get_time()/1000;
    uint8_t state = gpio_get_level(PWM_PIN);

    if(state == true){
        tL = now - lastNegEdgeTime;
        if(lastNegEdgeTime > 0 && lastPosEdgeTime > 0){
            Cppm = 5000 * (tH - 2) / (tH + tL - 4);
            xSemaphoreGive(s_sync);
        }
        lastPosEdgeTime = now;
    }
    else{
        tH = now - lastPosEdgeTime;
        lastNegEdgeTime = now;
    }
}

uint8_t getCheckSum(uint8_t *packet){
    int checksum = 0;
    for (int i = 1; i < 8; i++)
        checksum += packet[i];
    checksum = 0xff - checksum + 1;

    return checksum;
}