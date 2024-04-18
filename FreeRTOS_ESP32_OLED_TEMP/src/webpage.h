String webpage = "<html><head><title>ESP32 Sensor Data</title></head><body>";
  webpage += "<h1>ESP32 Sensor Data</h1>";
  webpage += "<p>Temperature: <span id='temp'>" + String(server_temperature) + "</span> C</p>";
  webpage += "<p>Humidity: <span id='humidity'>" + String(server_humidity) + "</span> % rh</p>";
  webpage += "<p>CO2: <span id='co2'>" + String(server_co2) + "</span> ppm</p>";
  webpage += "<script>setTimeout(function(){location.reload();}," + String(CONSUMER_TASK_PERIOD) + ");</script>";
  webpage += "</body></html>";