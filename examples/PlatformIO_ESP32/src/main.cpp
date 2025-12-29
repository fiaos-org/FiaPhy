#include <Arduino.h>
#include <FiaPhy.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* wifissid = "your_wifi_ssid";
const char* wifipass = "your_wifi_password";

const char* cloudendpoint = "https://api.fiaos.org/v1/solar-data";

Adafruit_BME280 bmeref(Wire, 0x76);
Adafruit_BME280 bmeflux(Wire, 0x77);

FiaPhy::SensorHub hub;
FiaPhy::PhysicsEngine engine;

const uint32_t samplegap = 1000;
const uint32_t uploadgap = 60000;
uint32_t lastsample = 0;
uint32_t lastupload = 0;
struct DataPoint {
    uint32_t timestamp;
    float ghi;
    float reftemp;
    float fluxtemp;
    float humidity;
    float pressure;
};

const uint8_t buffersize = 60;
DataPoint databuffer[buffersize];
uint8_t bufferindex = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== FiaPhy™ esp32 live sky ===\n");
    
    Wire.begin(21, 22);

    if (!bmeref.begin()) {
        Serial.println("ref bme280 not found");
        while(1) delay(1000);
    }
    
    if (!bmeflux.begin(0x77)) {
        Serial.println("flux bme280 not found");
        while(1) delay(1000);
    }

    bmeref.setSampling(Adafruit_BME280::MODE_FORCED,
                                            Adafruit_BME280::SAMPLING_X1,
                                            Adafruit_BME280::SAMPLING_X1,
                                            Adafruit_BME280::SAMPLING_X1,
                                            Adafruit_BME280::FILTER_OFF);
    
    bmeflux.setSampling(Adafruit_BME280::MODE_FORCED,
                                             Adafruit_BME280::SAMPLING_X1,
                                             Adafruit_BME280::SAMPLING_X1,
                                             Adafruit_BME280::SAMPLING_X1,
                                             Adafruit_BME280::FILTER_OFF);

    FiaPhy::initialize();
    hub.begin(2);
    
    FiaPhy::CalibrationParams tune;
    tune.thermaltime = 28.0f;
    tune.solarabsorb = 0.91f;
    engine.setCalibration(tune);

    Serial.print("Connecting to WiFi");
    WiFi.begin(wifissid, wifipass);
    
    while ( WiFi.status() != WL_CONNECTED ) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("\nstarting reads...\n");
    
    lastsample = millis();
    lastupload = millis();
}

void loop() {
    uint32_t nowms = millis();
    
    if(nowms - lastsample >= samplegap) {
        lastsample = nowms;
        
        bmeref.takeForcedMeasurement();
        bmeflux.takeForcedMeasurement();
        
        float reftemp = bmeref.readTemperature();
        float refhum = bmeref.readHumidity();
        float refpress = bmeref.readPressure() / 100.0f;
        
        float fluxtemp = bmeflux.readTemperature();
        float fluxhum = bmeflux.readHumidity();
        float fluxpress = bmeflux.readPressure() / 100.0f;
        
        hub.feedTemperature(reftemp, 0);
        hub.feedHumidity(refhum, 0);
        hub.feedPressure(refpress, 0);
        
        hub.feedTemperature(fluxtemp, 1);
        hub.feedHumidity(fluxhum, 1);
        hub.feedPressure(fluxpress, 1);
        
        if(hub.isFrameReady()) {
            const FiaPhy::SensorFrame* frames = hub.getFrames();
            FiaPhy::SolarFlux solar = engine.compute(frames, hub.getSensorCount());

            if (bufferindex < buffersize) {
                databuffer[bufferindex].timestamp = nowms;
                databuffer[bufferindex].ghi = solar.irradiance;
                databuffer[bufferindex].reftemp = frames[0].tempc;
                databuffer[bufferindex].fluxtemp = frames[1].tempc;
                databuffer[bufferindex].humidity = frames[0].humidity;
                databuffer[bufferindex].pressure = frames[0].pressure;
                bufferindex++;
            }
            
            Serial.printf("[%lu] ghi: %.1f W/m² | tref: %.2f°C | tflux: %.2f°C | dt: %.3f°C\n",
                                     nowms / 1000,
                                     solar.irradiance,
                                     frames[0].tempc,
                                     frames[1].tempc,
                                     frames[1].tempc - frames[0].tempc);
            
            hub.acknowledgeFrame();
        }
    }
    
    if(nowms - lastupload >= uploadgap && bufferindex > 0) {
        lastupload = nowms;
        uploadToCloud();
        bufferindex = 0;
    }
}

void uploadToCloud() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected - skipping upload");
        return;
    }
    
    Serial.println("\nuploading data...");

    StaticJsonDocument<4096> doc;
    doc["device_id"] = WiFi.macAddress();
    doc["firmware_version"] = FiaPhy::version();
    doc["sample_count"] = bufferindex;
    
    JsonArray data = doc.createNestedArray("data");
    
    for (uint8_t i = 0; i < bufferindex; i++) {
        JsonObject point = data.createNestedObject();
        point["timestamp"] = databuffer[i].timestamp;
        point["ghi"] = databuffer[i].ghi;
        point["temperature"] = databuffer[i].reftemp;
        point["humidity"] = databuffer[i].humidity;
        point["pressure"] = databuffer[i].pressure;
    }

    HTTPClient http;
    http.begin(cloudendpoint);
    http.addHeader("Content-Type", "application/json");
    
    String jsontext;
    serializeJson(doc, jsontext);
    
    int responsecode = http.POST(jsontext);
    
    if (responsecode > 0) {
        Serial.printf("upload ok (http %d)\n\n", responsecode);
    } else {
        Serial.printf("upload failed: %s\n\n", http.errorToString(responsecode).c_str());
    }
    
    http.end();
}
