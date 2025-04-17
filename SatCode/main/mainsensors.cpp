// #include <Wire.h>
// #include <TinyGPS++.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_HMC5883_U.h>
// #include <DHT.h>
// #include <DHT_U.h>

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

//   Serial.println(F("Sensors Initialized"));
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
//   Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s² ");
//   Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s² ");
//   Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s²");

//   // Magnetometer data
//   sensors_event_t event;
//   mag.getEvent(&event);
//   Serial.print("Mag X: "); Serial.print(event.magnetic.x); Serial.print(" µT ");
//   Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print(" µT ");
//   Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.println(" µT");

//   // DHT11 data
//   float humidity = dht.readHumidity();
//   float temperature = dht.readTemperature();
//   if (!isnan(humidity) && !isnan(temperature)) {
//     Serial.print("Humidity: "); Serial.print(humidity); Serial.print(" %  Temperature: ");
//     Serial.print(temperature); Serial.println(" °C");
//   } else {
//     Serial.println("Failed to read from DHT sensor!");
//   }

//   // Current Sensor (ACS712) data
//   int currentRaw = analogRead(CURRENT_PIN);
//   float currentVoltage = currentRaw * (3.3 / 4095.0);  // Convert raw reading to voltage (ESP32 ADC is 12-bit)
//   float current = (currentVoltage - ACS712_OFFSET) / ACS712_SENSITIVITY;
//   Serial.print("Current: "); Serial.print(current); Serial.println(" A");

//   // Voltage Sensor data
//   int voltageRaw = analogRead(VOLTAGE_PIN);
//   float voltageMeasured = voltageRaw * (3.3 / 4095.0); // Convert raw reading to voltage
//   float voltage = voltageMeasured * VOLTAGE_DIVIDER_RATIO;
//   Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V");

//   Serial.println("=====================");
//   delay(2000);
// }
