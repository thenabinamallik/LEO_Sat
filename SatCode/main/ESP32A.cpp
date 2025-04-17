// #include <WiFi.h>
// #include <WebServer.h>

// // Wi-Fi credentials for ESP32 A (Access Point)
// const char* ssid = "CodedHeart's LEO GSPL ";  // Name of the AP (Wi-Fi network)
// const char* password = "V17062003";  // Password for the AP

// WebServer server(80);

// void handleRoot() {
//   String roll = server.arg("roll");
//   String pitch = server.arg("pitch");
//   String latitude = server.arg("latitude");
//   String longitude = server.arg("longitude");
//   String magX = server.arg("magX");
//   String magY = server.arg("magY");
//   String magZ = server.arg("magZ");
//   String humidity = server.arg("humidity");
//   String temperature = server.arg("temperature");
//   String current = server.arg("current");
//   String voltage = server.arg("voltage");

//   // Print the received data to the Serial Monitor
//   Serial.println("Received Data:");
//   Serial.print("Roll: "); Serial.println(roll);
//   Serial.print("Pitch: "); Serial.println(pitch);
//   Serial.print("Latitude: "); Serial.println(latitude);
//   Serial.print("Longitude: "); Serial.println(longitude);
//   Serial.print("Magnetometer X: "); Serial.println(magX);
//   Serial.print("Magnetometer Y: "); Serial.println(magY);
//   Serial.print("Magnetometer Z: "); Serial.println(magZ);
//   Serial.print("Humidity: "); Serial.println(humidity);
//   Serial.print("Temperature: "); Serial.println(temperature);
//   Serial.print("Current: "); Serial.println(current);
//   Serial.print("Voltage: "); Serial.println(voltage);

//   server.send(200, "text/plain", "Data received successfully");
// }

// void setup() {
//   Serial.begin(115200);

//   // Set up Access Point
//   WiFi.softAP(ssid, password);
//   Serial.println("ESP32 A is now an Access Point.");
//   Serial.print("IP Address: ");
//   Serial.println(WiFi.softAPIP());

//   // Define the route for receiving the GET request
//   server.on("/", HTTP_GET, handleRoot);

//   // Start the server
//   server.begin();
// }

// void loop() {
//   server.handleClient();
// }
