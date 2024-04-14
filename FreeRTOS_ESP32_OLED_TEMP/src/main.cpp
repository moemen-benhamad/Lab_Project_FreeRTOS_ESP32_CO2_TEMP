#include "header.h"
#include "frames.h"

// TODO : DECIDE ON PERIOD AND Priority OF TASKS
// TODO : LOADING SCREEN WHEN NO MEASURMENTS

void setup() {
  Serial.begin(115200);

  dht.setup(DHT_PIN, DHTesp::DHT_TYPE);

  s1 = xSemaphoreCreateCounting( N, N );
  s2 = xSemaphoreCreateCounting( N, 0 );
  mutex = xSemaphoreCreateMutex();

  xTaskCreate(vDht22_Task, "dht22_Task", 4096, NULL, 1, NULL);
  xTaskCreate(vOled_Task, "oled_Task", 4096, NULL, 1, NULL);
  xTaskCreate(vCo2_Task, "co2_Task", 4096, NULL, 1, NULL);

  vTaskDelete(NULL);
}

void loop() {
  vTaskDelete(NULL);
}

void vDht22_Task(void *pvParameters) {

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
      printf("Temperature: %f, Humidity: %f\n", temperature, humidity);
    }

    // [PERIOD]
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DHT22_TASK_PERIOD)); // dht.getMinimumSamplingPeriod()
  }
  vTaskDelete(NULL);
}

void vOled_Task(void *pvParameters) {

  TickType_t xLastWakeTime = xTaskGetTickCount();
  SensorData data;
  float temperature = 0;
  float humidity = 0;
  int co2 = 0;
  int j = 0;
  int frame = 0;

  display.init();           
  display.clear();
      
  for (;;) {
    xSemaphoreTake(s2, portMAX_DELAY);
    data = buffer[j];
    j = (j+1)%N;
    xSemaphoreGive(s1);

    processSensorData(data, &temperature, &humidity, &co2);

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

    // [Period]
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(OLED_TASK_PERIOD));   
  }

  vTaskDelete(NULL);
}

void vCo2_Task(void *pvParameters){

  TickType_t xLastWakeTime = xTaskGetTickCount();
  int co2 = 0;
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
    if(uart_read_bytes(ESP_UART_PORT, &co2, 4, pdMS_TO_TICKS(1000)) == 4){
      printf("co2 reading recieved successfully from ESP32\n");
      printf("co2 [received]: %d\n", co2);
    }
    else {
      printf("Error receiving co2 reading from ESP32\n");
    }
    
    xSemaphoreTake(s1, portMAX_DELAY);
    xSemaphoreTake(mutex, portMAX_DELAY);
    buffer[i] = {CO2, (float)co2};
    i = (i+1)%N;
    xSemaphoreGive(mutex);
    xSemaphoreGive(s2);
    
    // [Period]
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CO2_TASK_PERIOD));
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

void display_buffer() {
    printf("Buffer Contents:\n");
    printf("Index\tType\t\tValue\n");
    printf("--------------------------------\n");
    for (int i = 0; i < 10; i++) {
      printf("%d\t", i);
      switch (buffer[i].type) {
        case TEMPERATURE:
          printf("Temperature\t");
          break;
        case HUMIDITY:
          printf("Humidity\t");
          break;
        case CO2:
          printf("CO2\t\t");
          break;
      }
      printf("%.2f\n", buffer[i].value);
    }
}