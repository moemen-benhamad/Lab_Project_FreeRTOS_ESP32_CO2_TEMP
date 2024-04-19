/*  ------------------------------------------------- WARNING --------------------------------------------------------------

  !CONNECT RED WIRE TO 5V WHEN POWERING WITH USB.
  !CONNECT RED WIRE TO VCC WHEN POWERING WITH AN EXERNAL POWER SOURCE (ESP32), other with the oled display won't turn on.

-------------------------------------------------------------------------------------------------------------------------- */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h> 
#include <SSD1306Wire.h> 
#include <HardwareSerial.h>
#include <DHTesp.h>
#include <WebServer.h>
#include "driver/uart.h"
#include "header.h"
#include "frames.h"

void setup() {
  Serial.begin(115200);

  dht.setup(DHT_PIN, DHTesp::DHT_TYPE);

  s1 = xSemaphoreCreateCounting( N, N );
  s2 = xSemaphoreCreateCounting( N, 0 );
  mutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(vdht22_Task, "dht22_Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(voled_Task, "oled_Task", 4096, NULL, 1, NULL , 0);
  xTaskCreatePinnedToCore(vuart_rx_task, "uart_rx_task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vweb_server_Task, "webServer_Task", 4096, NULL, 1, NULL, 1);

  vTaskDelete(NULL);
}

void loop() {
  vTaskDelete(NULL);
}

void vdht22_Task(void *pvParameters) {

  TickType_t xLastWakeTime = xTaskGetTickCount();
  float temperature = 0;
  float humidity = 0;

  for (;;) {                 
    temperature = dht.getTemperature();     
    humidity = dht.getHumidity();             
    if (isnan(temperature) || isnan(humidity)) {
      #ifdef SHOW_DHT22_ERROR_MSGS  
        printf("[DHT22] Failed!\n");
      #endif
    }
    else {
      xSemaphoreTake(s1, portMAX_DELAY);
      xSemaphoreTake(mutex, portMAX_DELAY);
      buffer[i] = {TEMPERATURE, temperature};
      i = (i+1)%N;
      buffer[i] = {HUMIDITY, humidity};
      i = (i+1)%N;
      xSemaphoreGive(mutex);
      xSemaphoreGive(s2);

      #ifdef SHOW_DHT22_MEASURMENTS
        printf("[DHT22] Temperature: %.2f\n", temperature);
        printf("[DHT22] Humidity: %.2f\n", humidity);
      #endif
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DHT22_TASK_PERIOD));
  }
  vTaskDelete(NULL);
}

void voled_Task(void *pvParameters) {

  TickType_t xLastWakeTime = xTaskGetTickCount();
  SensorData data;
  float temperature = 0;
  float humidity = 0;
  int co2 = 0;
  int j = 0;
  int frame = 0;

  display.init();           
  display.clear();
  #ifdef DISPLAY_ALWAYS_ON
    display.displayOn();
    display_isOn = true;
  #else
    display.displayOff();
    display_isOn = false;
  #endif
      
  for (;;) {
    xSemaphoreTake(s2, portMAX_DELAY);
    data = buffer[j];
    j = (j+1)%N;
    xSemaphoreGive(s1);

    processSensorData(data, &temperature, &humidity, &co2);

    // Server variables
    server_temperature = temperature; // MX???
    server_humidity = humidity;
    server_co2 = co2;

    if(display_isOn){ // MX???
      display.clear();
      display.setFont(ArialMT_Plain_16);    
      display.drawString(32, 2, "" + String(temperature) + " °C");
      display.setFont(ArialMT_Plain_10); 
      display.drawString(32, 18, "" + String(humidity) + " % rh");
      display.setFont(ArialMT_Plain_16); 
      display.drawRect(14, 40, 102 , 19); 
      display.drawString(40, 40, String(co2) + " ppm");
      display.drawXbm(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frames_temperature[frame]);
      display.drawXbm(0, 32, CO2_ICON_HEIGHT, CO2_ICON_WIDTH, co2_icon_bits);
      display.display();
      frame = (frame + 1) % FRAME_COUNT; 

      #ifdef DISPLAY_STATE_MSGS
        printf("[DISPLAY] Display Updated\n");
      #endif
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(OLED_TASK_PERIOD));   
  }

  vTaskDelete(NULL);
}

void vuart_rx_task(void *pvParameters){

  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xLastDisplayWakeTime = esp_timer_get_time();
  TickType_t now = esp_timer_get_time();
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

  for(;;){
    if(uart_read_bytes(ESP_UART_PORT, &uart_serial_data, 5, portMAX_DELAY) == 5){
      #ifdef ESP_UART_MSGS
        printf("[UART ESP32] Data recieved successfully\n");
        printf("  > Cppm : %d\n", uart_serial_data.co2_value);
        printf("  > Wake up signal : %d\n", uart_serial_data.wake_diplay_signal);
      #endif
    }
    else {
      #ifdef ESP_UART_MSGS
        printf("[UART ESP32] Error receiving data!\n");
      #endif
    }
    
    xSemaphoreTake(s1, portMAX_DELAY);
    xSemaphoreTake(mutex, portMAX_DELAY);
    buffer[i] = {CO2, (float) uart_serial_data.co2_value};
    i = (i+1)%N;
    xSemaphoreGive(mutex);
    xSemaphoreGive(s2);

    #ifndef DISPLAY_ALWAYS_ON
      if(!display_isOn){
        if(uart_serial_data.wake_diplay_signal == 1){
          xLastDisplayWakeTime = esp_timer_get_time()/1000;
          display.displayOn(); // MX???
          display_isOn = true;

          #ifdef DISPLAY_STATE_MSGS
            printf("[DISPLAY] Display ON\n");
          #endif

        }
      }
      else{
        now = esp_timer_get_time()/1000;
        if(now - xLastDisplayWakeTime >= DISPLAY_TIMEOUT){
          display_isOn = false;
          display.displayOff(); // MX???

          #ifdef DISPLAY_STATE_MSGS
            printf("[DISPLAY] Display OFF\n");
          #endif

        }
      }
    #endif
  
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(UART_RX_TASK_PERIOD));
  }
  vTaskDelete(NULL);
}

void vweb_server_Task(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Set up WiFi access point
  WiFi.softAP(ssid, password);
  WiFi.setHostname(hostname);
  
  Serial.print("[WEBSERVER] IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("[WEBSERVER] Hostname: ");
  Serial.println(WiFi.getHostname());

  server.on("/", handleRoot);
  server.begin();

  for(;;){
    server.handleClient();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(WEBSERVER_TASK_PERIOD));
  }
  vTaskDelete(NULL);
}

void processSensorData(SensorData data, float* temperature, float* humidity, int* co2) {
  switch (data.type) {
    case TEMPERATURE:
      *temperature = data.value;
      break;
    case HUMIDITY:
      *humidity = data.value;
      break;
    case CO2:
      *co2 = (int)data.value;
      break;
    default:
      break;
  }
}

void handleRoot() {
  String webpage = "<html><head><title>ESP32 Sensor Data</title></head><body>";
  webpage += "<h1>ESP32 Sensor Data</h1>";
  webpage += "<p>Temperature: <span id='temp'>" + String(server_temperature) + "</span> C</p>";
  webpage += "<p>Humidity: <span id='humidity'>" + String(server_humidity) + "</span> % rh</p>";
  webpage += "<p>CO2: <span id='co2'>" + String(server_co2) + "</span> ppm</p>";
  webpage += "<script>setTimeout(function(){location.reload();}," + String(CONSUMER_TASK_PERIOD) + ");</script>";
  webpage += "</body></html>";

  server.send(200, "text/html", webpage);
}