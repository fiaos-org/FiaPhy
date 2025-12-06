/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

// FiaPhy PlatformIO ESP32 Example - WiFi Data Logging

#include <Arduino.h>
#include <FiaPhy.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* WIFI_SSID = "your_wifi_ssid";
const char* WIFI_PASSWORD = "your_wifi_password";

// Cloud endpoint
const char* CLOUD_ENDPOINT = "https://api.fiaos.org/v1/solar-data";

// BME280 sensors
Adafruit_BME280 bme_ref(Wire, 0x76);
Adafruit_BME280 bme_flux(Wire, 0x77);

// FiaPhy objects
FiaPhy::SensorHub hub;
FiaPhy::PhysicsEngine engine;

// Timing intervals
const uint32_t SAMPLE_INTERVAL_MS = 1000;
const uint32_t UPLOAD_INTERVAL_MS = 60000;
uint32_t last_sample_time = 0;
uint32_t last_upload_time = 0;

// Data buffer
struct DataPoint {
    uint32_t timestamp;
    float ghi;
    float temp_ref;
    float temp_flux;
    float humidity;
    float pressure;
};

const uint8_t BUFFER_SIZE = 60;
DataPoint data_buffer[BUFFER_SIZE];
uint8_t buffer_index = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== FiaPhy ESP32 Solar Monitor ===\n");
    
    // I2C init (SDA=21, SCL=22)
    Wire.begin(21, 22);
    
    // Sensor init
    if(!bme_ref.begin()) {
        Serial.println("ERROR: Reference BME280 not found");
        while(1) delay(1000);
    }
    
    if(!bme_flux.begin(0x77)) {
        Serial.println("ERROR: Flux BME280 not found");
        while(1) delay(1000);
    }
    
    // Sensor config
    bme_ref.setSampling(Adafruit_BME280::MODE_FORCED,
                                            Adafruit_BME280::SAMPLING_X1,
                                            Adafruit_BME280::SAMPLING_X1,
                                            Adafruit_BME280::SAMPLING_X1,
                                            Adafruit_BME280::FILTER_OFF);
    
    bme_flux.setSampling(Adafruit_BME280::MODE_FORCED,
                                             Adafruit_BME280::SAMPLING_X1,
                                             Adafruit_BME280::SAMPLING_X1,
                                             Adafruit_BME280::SAMPLING_X1,
                                             Adafruit_BME280::FILTER_OFF);
    
    // FiaPhy init
    FiaPhy::initialize();
    hub.begin(2);
    
    FiaPhy::CalibrationParams calib;
    calib.thermal_time_constant_s = 28.0f;
    calib.solar_absorptivity = 0.91f;
    engine.setCalibration(calib);
    
    // WiFi connect
    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while(WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("\nStarting measurements...\n");
    
    last_sample_time = millis();
    last_upload_time = millis();
}

void loop() {
    uint32_t current_time = millis();
    
    // Sample sensors
    if(current_time - last_sample_time >= SAMPLE_INTERVAL_MS) {
        last_sample_time = current_time;
        
        bme_ref.takeForcedMeasurement();
        bme_flux.takeForcedMeasurement();
        
        float temp_ref = bme_ref.readTemperature();
        float humid_ref = bme_ref.readHumidity();
        float press_ref = bme_ref.readPressure() / 100.0f;
        
        float temp_flux = bme_flux.readTemperature();
        float humid_flux = bme_flux.readHumidity();
        float press_flux = bme_flux.readPressure() / 100.0f;
        
        hub.feedTemperature(temp_ref, 0);
        hub.feedHumidity(humid_ref, 0);
        hub.feedPressure(press_ref, 0);
        
        hub.feedTemperature(temp_flux, 1);
        hub.feedHumidity(humid_flux, 1);
        hub.feedPressure(press_flux, 1);
        
        // Process frame
        if(hub.isFrameReady()) {
            const FiaPhy::SensorFrame* frames = hub.getFrames();
            FiaPhy::SolarFlux solar = engine.compute(frames, hub.getSensorCount());
            
            // Store in buffer
            if(buffer_index < BUFFER_SIZE) {
                data_buffer[buffer_index].timestamp = current_time;
                data_buffer[buffer_index].ghi = solar.irradiance_Wm2;
                data_buffer[buffer_index].temp_ref = frames[0].temperature_C;
                data_buffer[buffer_index].temp_flux = frames[1].temperature_C;
                data_buffer[buffer_index].humidity = frames[0].humidity_RH;
                data_buffer[buffer_index].pressure = frames[0].pressure_hPa;
                buffer_index++;
            }
            
            Serial.printf("[%lu] GHI: %.1f W/m² | T_ref: %.2f°C | T_flux: %.2f°C | ΔT: %.3f°C\n",
                                     current_time / 1000,
                                     solar.irradiance_Wm2,
                                     frames[0].temperature_C,
                                     frames[1].temperature_C,
                                     frames[1].temperature_C - frames[0].temperature_C);
            
            hub.acknowledgeFrame();
        }
    }
    
    // Upload to cloud
    if(current_time - last_upload_time >= UPLOAD_INTERVAL_MS && buffer_index > 0) {
        last_upload_time = current_time;
        uploadToCloud();
        buffer_index = 0;
    }
}

void uploadToCloud() {
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected - skipping upload");
        return;
    }
    
    Serial.println("\nUploading data to cloud...");
    
    // Build JSON payload
    StaticJsonDocument<4096> doc;
    doc["device_id"] = WiFi.macAddress();
    doc["firmware_version"] = FiaPhy::version();
    doc["sample_count"] = buffer_index;
    
    JsonArray data = doc.createNestedArray("data");
    
    for(uint8_t i = 0; i < buffer_index; i++) {
        JsonObject point = data.createNestedObject();
        point["timestamp"] = data_buffer[i].timestamp;
        point["ghi_Wm2"] = data_buffer[i].ghi;
        point["temperature_C"] = data_buffer[i].temp_ref;
        point["humidity_RH"] = data_buffer[i].humidity;
        point["pressure_hPa"] = data_buffer[i].pressure;
    }
    
    // HTTP POST
    HTTPClient http;
    http.begin(CLOUD_ENDPOINT);
    http.addHeader("Content-Type", "application/json");
    
    String json_string;
    serializeJson(doc, json_string);
    
    int response_code = http.POST(json_string);
    
    if(response_code > 0) {
        Serial.printf("Upload successful (HTTP %d)\n\n", response_code);
    } else {
        Serial.printf("Upload failed: %s\n\n", http.errorToString(response_code).c_str());
    }
    
    http.end();
}
