/*
 * IoT Data Engineering: Complete Sensors Chain
 * LDR (Power Law Calibrated) + BMP280 Sensor
 * MQTT Publishing with JSON Format
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

// ==================== CONFIGURATION ====================

// WiFi Credentials
const char* ssid = "Pixel_5005";  
const char* password = "johnriecken";

// MQTT Broker Configuration
const char* mqtt_server = "test.mosquitto.org";//"141.79.71.175";  
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_Sensor_Group_04";

// MQTT Topics
const char* topic_sensor_data = "iot/sensors/group04/data";
const char* topic_status = "iot/sensors/group04/status";
const char* topic_config = "iot/sensors/group04/config";

// Pin Definitions
#define LDR_PIN 34           // ADC1_CH6
#define LED_PIN 2            // Built-in LED
#define SDA_PIN 21
#define SCL_PIN 22

// LDR Calibration Constants (Power Law: Lux = A * ADC^B + C)
// These values must be determined from your calibration

//Update A and B with calibration data
const float LDR_COEFF_A = 2.85e+08;  // Example value
const float LDR_COEFF_B = -1.64;       // Example value (gamma)
const float LDR_COEFF_C = 0.0;        // Offset

// Sampling Configuration
const unsigned long SAMPLE_INTERVAL = 20000;  // 20 seconds
const int ADC_SAMPLES = 20;                   // Averaging samples

// NTP Configuration
const char* ntp_server = "pool.ntp.org";
const long gmt_offset_sec = 3600;             // GMT+1
const int daylight_offset_sec = 3600;         // DST

// ==================== GLOBAL OBJECTS ====================

Adafruit_BMP280 bmp;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

// Data structure
struct SensorData {
  float temperature;
  float pressure;
  float altitude;
  float lux;
  int raw_adc;
  unsigned long timestamp;
  String iso_time;
};

SensorData currentData;
unsigned long lastSampleTime = 0;
unsigned long lastReconnectAttempt = 0;

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=================================");
  Serial.println("IoT Data Engineering - Sensor Node");
  Serial.println("=================================\n");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize BMP280
  if (!bmp.begin(0x76)) {  // Try 0x77 if 0x76 fails
    Serial.println("ERROR: BMP280 sensor not found!");
    Serial.println("Check wiring and I2C address");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  
  Serial.println("BMP280 sensor initialized");
  
  // Configure BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode
                  Adafruit_BMP280::SAMPLING_X2,     // Temp oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time
  
  // Configure ADC for LDR
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db);  // 0-3.3V range
  
  Serial.println("LDR ADC configured");
  
  // Connect to WiFi
  connectWiFi();
  
  // Initialize NTP
  configTime(gmt_offset_sec, daylight_offset_sec, ntp_server);
  Serial.println("NTP time synchronization started");
  
  // Configure MQTT
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  mqtt_client.setBufferSize(512);  // Increase buffer for JSON
  
  Serial.println("\n=== System Ready ===\n");
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

// ==================== MAIN LOOP ====================

void loop() {
  // Maintain MQTT connection
  if (!mqtt_client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnectMQTT()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    mqtt_client.loop();
  }
  
  // Sample sensors at specified interval
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    
    // Visual indicator
    digitalWrite(LED_PIN, HIGH);
    
    // Read sensors
    readSensors();
    
    // Print to serial
    printSensorData();
    
    // Publish to MQTT
    publishSensorData();
    
    digitalWrite(LED_PIN, LOW);
  }
}

// ==================== SENSOR READING ====================

void readSensors() {
  // Read BMP280
  currentData.temperature = bmp.readTemperature();
  currentData.pressure = bmp.readPressure() / 100.0F;  // Convert to hPa
  currentData.altitude = bmp.readAltitude(1013.25);     // Sea level pressure
  
  // Read LDR with averaging
  long adcSum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    adcSum += analogRead(LDR_PIN);
    delay(20);
  }
  currentData.raw_adc = adcSum / ADC_SAMPLES;
  
  // Convert ADC to Lux using power law calibration
  currentData.lux = calculateLux(currentData.raw_adc);
  
  // Get timestamp
  currentData.timestamp = millis();
  currentData.iso_time = getISOTime();
}

float calculateLux(int adc_value) {
  // Handle edge cases
  if (adc_value <= 0) return 0.0;
  if (adc_value >= 4095) return 2.85e+08;  // Max lux
  
  // Apply power law: Lux = A * ADC^B + C
  float lux = LDR_COEFF_A * pow(adc_value, LDR_COEFF_B) + LDR_COEFF_C;
  
  // Ensure non-negative
  if (lux < 0) lux = 0;
  
  return lux;
}

// ==================== WIFI CONNECTION ====================

void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n WiFi connection failed!");
  }
}

// ==================== MQTT CONNECTION ====================

boolean reconnectMQTT() {
  Serial.print("Attempting MQTT connection... ");
  
  if (mqtt_client.connect(mqtt_client_id)) {
    Serial.println("connected");
    
    // Subscribe to config topic
    mqtt_client.subscribe(topic_config);
    
    // Publish status
    publishStatus("online");
    
    return true;
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt_client.state());
    return false;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  
  // Parse incoming configuration messages
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (!error && strcmp(topic, topic_config) == 0) {
    // Handle configuration updates
    Serial.println("Configuration update received");
  }
}

// ==================== DATA PUBLISHING ====================

void publishSensorData() {
  if (!mqtt_client.connected()) return;
  
  // Create JSON document
  StaticJsonDocument<512> doc;
  
  // Device info
  doc["device_id"] = mqtt_client_id;
  doc["timestamp"] = currentData.timestamp;
  doc["iso_time"] = currentData.iso_time;
  
  // Sensor data
  JsonObject sensors = doc.createNestedObject("sensors");
  
  // BMP280 data
  JsonObject bmp280 = sensors.createNestedObject("bmp280");
  bmp280["temperature"] = round(currentData.temperature * 100) / 100.0;
  bmp280["pressure"] = round(currentData.pressure * 100) / 100.0;
  bmp280["altitude"] = round(currentData.altitude * 10) / 10.0;
  bmp280["unit_temp"] = "°C";
  bmp280["unit_pressure"] = "hPa";
  bmp280["unit_altitude"] = "m";
  
  // LDR data
  JsonObject ldr = sensors.createNestedObject("ldr");
  ldr["lux"] = round(currentData.lux * 10) / 10.0;
  ldr["raw_adc"] = currentData.raw_adc;
  ldr["unit"] = "lux";
  
  // System info
  JsonObject system = doc.createNestedObject("system");
  system["wifi_rssi"] = WiFi.RSSI();
  system["free_heap"] = ESP.getFreeHeap();
  system["uptime"] = millis() / 1000;
  
  // Serialize and publish
  char jsonBuffer[512];
  size_t len = serializeJson(doc, jsonBuffer);
  
  if (mqtt_client.publish(topic_sensor_data, jsonBuffer, len)) {
    Serial.println(" Data published to MQTT");
  } else {
    Serial.println(" MQTT publish failed");
  }
}

void publishStatus(const char* status) {
  StaticJsonDocument<128> doc;
  doc["device_id"] = mqtt_client_id;
  doc["status"] = status;
  doc["timestamp"] = millis();
  doc["ip"] = WiFi.localIP().toString();
  
  char jsonBuffer[128];
  serializeJson(doc, jsonBuffer);
  mqtt_client.publish(topic_status, jsonBuffer, true);  // Retained message
}

// ==================== UTILITY FUNCTIONS ====================

String getISOTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "Time not set";
  }
  
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
  return String(buffer);
}

void printSensorData() {
  Serial.println("\n--- Sensor Reading ---");
  Serial.print("Time: ");
  Serial.println(currentData.iso_time);
  Serial.println();
  
  Serial.println("BMP280:");
  Serial.print("  Temperature: ");
  Serial.print(currentData.temperature);
  Serial.println(" °C");
  Serial.print("  Pressure: ");
  Serial.print(currentData.pressure);
  Serial.println(" hPa");
  Serial.print("  Altitude: ");
  Serial.print(currentData.altitude);
  Serial.println(" m");
  Serial.println();
  
  Serial.println("LDR:");
  Serial.print("  Light Intensity: ");
  Serial.print(currentData.lux);
  Serial.println(" lux");
  Serial.print("  Raw ADC: ");
  Serial.println(currentData.raw_adc);
  Serial.println("---------------------\n");
}