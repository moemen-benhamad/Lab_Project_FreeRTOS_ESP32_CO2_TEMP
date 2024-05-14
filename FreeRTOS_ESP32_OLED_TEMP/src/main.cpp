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
  server_mx = xSemaphoreCreateMutex();
  display_mx = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(vdht22_task, "dht22_Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(voled_task, "oled_Task", 4096, NULL, 1, NULL , 0);
  xTaskCreatePinnedToCore(vuart_rx_task, "uart_rx_task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vweb_server_task, "webServer_Task", 4096, NULL, 1, NULL, 1);

  vTaskDelete(NULL);
}

void loop() {
  vTaskDelete(NULL);
}

void vdht22_task(void *pvParameters) {

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

void voled_task(void *pvParameters) {

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
    xSemaphoreTake(display_mx, portMAX_DELAY);
    display_isOn = true;
    xSemaphoreGive(display_mx);

  #else
    display.displayOff();
    xSemaphoreTake(display_mx, portMAX_DELAY);
    display_isOn = false;
    xSemaphoreGive(display_mx);
  #endif
      
  for (;;) {
    xSemaphoreTake(s2, portMAX_DELAY);
    data = buffer[j];
    j = (j+1)%N;
    xSemaphoreGive(s1);

    processSensorData(data, &temperature, &humidity, &co2);

    // NEW
    xSemaphoreTake(server_mx, portMAX_DELAY);
    server_data.temperature = temperature;
    server_data.humidity = humidity;
    server_data.co2 = co2;
    xSemaphoreGive(server_mx);
    /*
    server_temperature = temperature;
    server_humidity = humidity;
    server_co2 = co2;
    */

    xSemaphoreTake(display_mx, portMAX_DELAY);
    if(display_isOn){
      xSemaphoreGive(display_mx);
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
    else{
      xSemaphoreGive(display_mx);
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
      xSemaphoreTake(display_mx, portMAX_DELAY);
      if(!display_isOn){
        xSemaphoreGive(display_mx);
        if(uart_serial_data.wake_diplay_signal == 1){
          xLastDisplayWakeTime = esp_timer_get_time()/1000;
          display.displayOn();
          xSemaphoreTake(display_mx, portMAX_DELAY);
          display_isOn = true;
          xSemaphoreGive(display_mx);

          #ifdef DISPLAY_STATE_MSGS
            printf("[DISPLAY] Display ON\n");
          #endif

        }
      }
      else{
        xSemaphoreGive(display_mx);
        now = esp_timer_get_time()/1000;
        if(now - xLastDisplayWakeTime >= DISPLAY_TIMEOUT){
          xSemaphoreTake(display_mx, portMAX_DELAY);
          display_isOn = false;
          xSemaphoreGive(display_mx);
          display.displayOff();

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

void vweb_server_task(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();

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
  float temperature, humidity, co2;
  ServerData data;
  xSemaphoreTake(server_mx, portMAX_DELAY);
  data = server_data;
  xSemaphoreGive(server_mx);
  temperature = data.temperature;
  humidity = data.humidity;
  co2 =  data.co2;

  String webpage = "<html><head><title>ESP32 Sensor Data</title></head><body>";
  webpage += "<h1>ESP32 Sensor Data</h1>";
  webpage += "<p>Temperature: <span id='temp'>" + String(temperature) + "</span> C</p>";
  webpage += "<p>Humidity: <span id='humidity'>" + String(humidity) + "</span> % rh</p>";
  webpage += "<p>CO2: <span id='co2'>" + String(co2) + "</span> ppm</p>";
  webpage += "<script>setTimeout(function(){location.reload();}," + String(CONSUMER_TASK_PERIOD) + ");</script>";
  webpage += "</body></html>";

  server.send(200, "text/html", webpage);
}