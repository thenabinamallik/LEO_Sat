// #include <Wire.h>
// #include <TinyGPS++.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_HMC5883_U.h>
// #include <DHT.h>
// #include <DHT_U.h>
// #include <WiFi.h>
// #include <HTTPClient.h>

// // Define GPS
// TinyGPSPlus gps;
// HardwareSerial GPS_Serial(1); // Serial1 for GPS

// // Define MPU6050
// Adafruit_MPU6050 mpu;

// // Define HMC5883L Magnetometer
// Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// // Define DHT11
// #define DHTPIN 14        
// #define DHTTYPE DHT11   
// DHT dht(DHTPIN, DHTTYPE);

// // Define Voltage and Current Sensor Pins
// #define VOLTAGE_PIN 33
// #define CURRENT_PIN 35

// // Sensor parameters (adjust for your sensor type)
// const float ACS712_OFFSET = 2.5;    // Midpoint voltage (0A)
// const float ACS712_SENSITIVITY = 0.185; // Sensitivity in V/A for ACS712-5A

// const float VOLTAGE_DIVIDER_RATIO = 5.0; // Adjust based on R1 and R2 values

// // Wi-Fi credentials
// const char* ssid = "CodedHeart's LEO GSPL";  // ESP32 A's Access Point SSID
// const char* password = "V17062003";  // ESP32 A's Access Point Password

// // IP address of ESP32 A (Access Point)
// const char* serverName = "http://192.168.4.1/";  // Replace with ESP32 A's IP

// void setup() {
//   Serial.begin(115200);
  
//   // Initialize GPS
//   GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); 

//   // Initialize MPU6050
//   if (!mpu.begin()) {
//     Serial.println("MPU6050 not found!");
//     while (1);
//   }

//   // Initialize Magnetometer
//   if (!mag.begin()) {
//     Serial.println("HMC5883L not found!");
//     while (1);
//   }

//   // Initialize DHT11
//   dht.begin();

//   // Connect to Wi-Fi
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(1000);
//     Serial.println("Connecting to WiFi...");
//   }

//   Serial.println("Connected to Wi-Fi");
// }

// void loop() {
//   // GPS data
//   while (GPS_Serial.available() > 0) {
//     gps.encode(GPS_Serial.read());
//     if (gps.location.isUpdated()) {
//       Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
//       Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
//     }
//   }

//   // MPU6050 data
//   sensors_event_t accel, gyro, temp;
//   mpu.getEvent(&accel, &gyro, &temp);

//   // Calculate Roll and Pitch
//   float roll = atan2(accel.acceleration.y, sqrt(accel.acceleration.x * accel.acceleration.x + accel.acceleration.z * accel.acceleration.z)) * 180 / PI;
//   float pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180 / PI;

//   Serial.print("Roll: "); Serial.print(roll); Serial.print("° ");
//   Serial.print("Pitch: "); Serial.print(pitch); Serial.println("°");

//   // Read Magnetometer data
//   sensors_event_t event;
//   mag.getEvent(&event);
//   float magX = event.magnetic.x;
//   float magY = event.magnetic.y;
//   float magZ = event.magnetic.z;

//   // Read DHT11 data
//   float humidity = dht.readHumidity();
//   float temperature = dht.readTemperature();
  
//   // Current Sensor (ACS712) data
//   int currentRaw = analogRead(CURRENT_PIN);
//   float currentVoltage = currentRaw * (3.3 / 4095.0);  // Convert raw reading to voltage (ESP32 ADC is 12-bit)
//   float current = (currentVoltage - ACS712_OFFSET) / ACS712_SENSITIVITY;
  
//   // Voltage Sensor data
//   int voltageRaw = analogRead(VOLTAGE_PIN);
//   float voltageMeasured = voltageRaw * (3.3 / 4095.0); // Convert raw reading to voltage
//   float voltage = voltageMeasured * VOLTAGE_DIVIDER_RATIO;

//   // Send all data to ESP32 A via HTTP GET request
//   HTTPClient http;

//   // Explicitly convert serverName to String before concatenating
//   String url = String(serverName) + "?roll=" + String(roll) +
//                                  "&pitch=" + String(pitch) +
//                                  "&latitude=" + String(gps.location.lat(), 6) +
//                                  "&longitude=" + String(gps.location.lng(), 6) +
//                                  "&magX=" + String(magX) +
//                                  "&magY=" + String(magY) +
//                                  "&magZ=" + String(magZ) +
//                                  "&humidity=" + String(humidity) +
//                                  "&temperature=" + String(temperature) +
//                                  "&current=" + String(current) +
//                                  "&voltage=" + String(voltage);
                         
//   http.begin(url);  // Begin the HTTP request

//   int httpResponseCode = http.GET();  // Send GET request

//   if (httpResponseCode > 0) {
//     String response = http.getString();  // Get the response from ESP32 A
//     Serial.println("Response from ESP32 A: " + response);
//   } else {
//     Serial.println("Error in HTTP request");
//   }

//   http.end();  // End the HTTP request

//   Serial.println("=====================");
//   delay(2000);  // Wait 2 seconds before sending the next set of data
// }
// // 