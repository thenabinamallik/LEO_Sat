#include <WiFi.h>
#include <FirebaseESP32.h>
#include <HTTPClient.h>
#include <base64.h>  
#include <ArduinoJson.h>

const char* ssid = "CodedHeart LEO GSPL";
const char* password = "v17062003";

#define FIREBASE_HOST "https://leo-sat-7c5e5-default-rtdb.asia-southeast1.firebasedatabase.app/" 
#define FIREBASE_AUTH "AIzaSyC1XBTXo66oN5R2xgch8thx875lxq4n8Ow"          

FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

// Reduce intervals to limit Firebase data storage
const unsigned long sensorDataInterval = 10000;
const unsigned long imageFetchInterval = 60000;
unsigned long lastSensorDataTime = 0;
unsigned long lastImageTime = 0;

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(500);
    setupFirebase();
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastSensorDataTime >= sensorDataInterval) {
        sendSensorDataToFirebase();
        lastSensorDataTime = currentMillis;
    }
    if (currentMillis - lastImageTime >= imageFetchInterval) {
        fetchAndSendImageToFirebase();
        lastImageTime = currentMillis;
    }
}

void setupFirebase() {
    config.host = FIREBASE_HOST;
    config.api_key = FIREBASE_AUTH;
    auth.user.email = "nabinamallik2003@gmail.com";
    auth.user.password = "nabina@2003";
    Firebase.begin(&config, &auth);
}

void sendSensorDataToFirebase() {
    String sensorData = fetchSensorDataFromESP32B();
    DynamicJsonDocument doc(512);  // Smaller JSON buffer

    if (deserializeJson(doc, sensorData)) return; // Exit on error

    Firebase.setFloat(firebaseData, "/sensorData/gpsData/latitude", doc["latitude"]);
    Firebase.setFloat(firebaseData, "/sensorData/gpsData/longitude", doc["longitude"]);
    Firebase.setFloat(firebaseData, "/sensorData/gpsData/altitude", doc["altitude"]);
}

String fetchSensorDataFromESP32B() {
    HTTPClient http;
    String url = "http://192.168.43.125/getData";
    http.begin(url);
    int httpCode = http.GET();
    String sensorData = httpCode == HTTP_CODE_OK ? http.getString() : "";
    http.end();
    return sensorData;
}

void fetchAndSendImageToFirebase() {
    String imageBase64 = fetchImageFromCamera();
    Firebase.setString(firebaseData, "/imageData/image", imageBase64);
}

String fetchImageFromCamera() {
    HTTPClient http;
    http.begin("http://192.168.43.50/640x480.jpg");
    int httpCode = http.GET();
    String base64Image = "";
    if (httpCode == HTTP_CODE_OK) {
        WiFiClient *stream = http.getStreamPtr();
        uint8_t buffer[256];
        int len;
        while ((len = stream->readBytes(buffer, sizeof(buffer))) > 0) {
            base64Image += base64::encode(buffer, len);
        }
    }
    http.end();
    return base64Image;
}
