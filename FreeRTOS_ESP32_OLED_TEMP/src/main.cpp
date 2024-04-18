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
  xTaskCreatePinnedToCore(voled_Task, "oled_Task", 4096, NULL, 1, &voled_task_handle , 0);
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
      printf("Failed to read from DHT sensor!\n");
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
      printf("Temperature: %.2f, Humidity: %.2f\n", temperature, humidity);
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
  display.displayOff();

  vTaskSuspend(NULL);
      
  for (;;) {
    xSemaphoreTake(s2, portMAX_DELAY);
    data = buffer[j];
    j = (j+1)%N;
    xSemaphoreGive(s1);

    processSensorData(data, &temperature, &humidity, &co2);

    // Server variables
    server_temperature = temperature;
    server_humidity = humidity;
    server_co2 = co2;

    display.clear();
    display.setFont(ArialMT_Plain_16);    
    display.drawString(32, 2, "" + String(temperature) + " °C");
    display.setFont(ArialMT_Plain_10); 
    display.drawString(32, 18, "" + String(humidity) + " % rh");
    display.setFont(ArialMT_Plain_16); 
    display.drawRect(14, 40, 102 , 19); //36
    display.drawString(40, 40, String(co2) + " ppm");
    display.drawXbm(0, 0, FRAME_WIDTH, FRAME_HEIGHT, frames_temperature[frame]);
    display.drawXbm(0, 32, CO2_ICON_HEIGHT, CO2_ICON_WIDTH, co2_icon_bits);
    display.display();
    frame = (frame + 1) % FRAME_COUNT; 

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(OLED_TASK_PERIOD));   
  }

  vTaskDelete(NULL);
}

void vuart_rx_task(void *pvParameters){

  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xLastSuspendTime;
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
      printf("UART data recieved successfully from ESP32 : %d | %d\n", uart_serial_data.wake_diplay_signal, uart_serial_data.co2_value);
    }
    else {
      printf("Error receiving co2 reading from ESP32\n");
    }

    if(uart_serial_data.wake_diplay_signal == 1){
      display.displayOn(); // MX?
      vTaskResume(voled_task_handle);
      xLastSuspendTime = xTaskGetTickCount();
      vTaskDelayUntil(&xLastSuspendTime, pdMS_TO_TICKS(DISPLAY_TIMEOUT)); // ??
      vTaskSuspend(voled_task_handle);
      display.displayOff();
    }
    
    xSemaphoreTake(s1, portMAX_DELAY);
    xSemaphoreTake(mutex, portMAX_DELAY);
    buffer[i] = {CO2, (float) uart_serial_data.co2_value};
    i = (i+1)%N;
    xSemaphoreGive(mutex);
    xSemaphoreGive(s2);
    
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(UART_RX_TASK_PERIOD));
  }
  vTaskDelete(NULL);
}

void vweb_server_Task(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // Set up WiFi access point
  WiFi.softAP(ssid, password);
  WiFi.setHostname(hostname);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(IP);

  server.on("/", handleRoot);
  server.begin();

  for(;;){
    server.handleClient();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
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
      printf("Unknown data type!\n");
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