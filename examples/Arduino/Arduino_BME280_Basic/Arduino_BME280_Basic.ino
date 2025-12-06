/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

/**
 * @file Arduino_BME280_Basic.ino
 * @brief FiaPhy DTDSS example with dual BME280 sensors
 * 
 * Hardware: Arduino + 2x BME280 (I2C 0x76, 0x77)
 * Wiring: SCL→A5, SDA→A4, VCC→3.3V, GND→GND
 * 
 * @license MIT License
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <FiaPhy.h>

// Sensors
Adafruit_BME280 bme_ref;  // 0x76
Adafruit_BME280 bme_flux; // 0x77

// FiaPhy system
FiaPhy::DTDSS system;

// Location configuration
const float LATITUDE = 6.9271f;
const float LONGITUDE = 79.8612f;
const float ALTITUDE_M = 5.0f;

// Sampling
const unsigned long SAMPLE_INTERVAL_MS = 1000;
unsigned long last_sample_time = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("FiaPhy DTDSS - Arduino BME280");
    Serial.println();
    
    Wire.begin();
    
    // Initialize sensors
    if (!bme_ref.begin(0x76)) {
        Serial.println("ERROR: Reference BME280 not found at 0x76");
        while (1) delay(100);
    }
    Serial.println("Reference sensor initialized (0x76)");
    
    if (!bme_flux.begin(0x77)) {
        Serial.println("ERROR: Flux BME280 not found at 0x77");
        while (1) delay(100);
    }
    Serial.println("Flux sensor initialized (0x77)");
    
    // Configure forced mode sampling
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
    
    Serial.println("Sensor sampling configured");
    
    // Configure FiaPhy
    if (system.configure(LATITUDE, LONGITUDE, ALTITUDE_M)) {
        Serial.println("FiaPhy system configured");
        Serial.print("Location: ");
        Serial.print(LATITUDE, 4);
        Serial.print("N, ");
        Serial.print(LONGITUDE, 4);
        Serial.println("E");
        Serial.print("Altitude: ");
        Serial.print(ALTITUDE_M);
        Serial.println(" m");
    } else {
        Serial.println("ERROR: System configuration failed");
        while (1) delay(100);
    }
    
    Serial.println();
    Serial.println("System ready");
    Serial.println();
    
    delay(2000);
}

void loop() {
    unsigned long current_time = millis();
    
    if (current_time - last_sample_time >= SAMPLE_INTERVAL_MS) {
        last_sample_time = current_time;
        
        // Trigger measurements
        bme_ref.takeForcedMeasurement();
        bme_flux.takeForcedMeasurement();
        delay(10);
        
        // Read sensors
        float ref_temp = bme_ref.readTemperature();
        float ref_humidity = bme_ref.readHumidity();
        float ref_pressure = bme_ref.readPressure() / 100.0f;
        
        float flux_temp = bme_flux.readTemperature();
        float flux_humidity = bme_flux.readHumidity();
        float flux_pressure = bme_flux.readPressure() / 100.0f;
        
        // Display readings
        Serial.println("--- Sensor Readings ---");
        Serial.print("REF:  T=");
        Serial.print(ref_temp, 2);
        Serial.print("C  H=");
        Serial.print(ref_humidity, 1);
        Serial.print("%  P=");
        Serial.print(ref_pressure, 2);
        Serial.println(" hPa");
        
        Serial.print("FLUX: T=");
        Serial.print(flux_temp, 2);
        Serial.print("C  H=");
        Serial.print(flux_humidity, 1);
        Serial.print("%  P=");
        Serial.print(flux_pressure, 2);
        Serial.println(" hPa");
        
        Serial.print("Delta T = ");
        Serial.print(flux_temp - ref_temp, 3);
        Serial.println("C");
        Serial.println();
        
        // Feed data to system
        system.feedReferenceTemperature(ref_temp, 0);
        system.feedReferenceHumidity(ref_humidity, 0);
        system.feedReferencePressure(ref_pressure, 0);
        
        system.feedFluxTemperature(flux_temp, 0);
        system.feedFluxHumidity(flux_humidity, 0);
        system.feedFluxPressure(flux_pressure, 0);
        
        // Compute radiation
        if (system.isFrameReady()) {
            Serial.println("--- Solar Radiation ---");
            
            FiaPhy::RadiationResult result = system.compute();
            
            if (result.valid) {
                Serial.print("GHI: ");
                Serial.print(result.ghi_Wm2, 1);
                Serial.println(" W/m2");
                
                Serial.print("Baseline GHI: ");
                Serial.print(result.baseline_ghi_Wm2, 1);
                Serial.println(" W/m2");
                
                Serial.print("Cloud Fraction: ");
                Serial.print(result.cloud_proxy * 100.0f, 1);
                Serial.println("%");
                
                Serial.println();
                Serial.println("--- Atmospheric State ---");
                
                Serial.print("Air Density: ");
                Serial.print(result.air_density_kg_m3, 4);
                Serial.println(" kg/m3");
                
                Serial.print("Vapor Pressure: ");
                Serial.print(result.vapor_pressure_hPa, 2);
                Serial.println(" hPa");
                
                Serial.print("Temp Differential: ");
                Serial.print(result.temp_differential_C, 3);
                Serial.println(" C");
                
                Serial.print("Temp Derivative: ");
                Serial.print(result.temp_derivative_C_s, 5);
                Serial.println(" C/s");
                
            } else {
                Serial.println("Computation failed");
            }
        } else {
            Serial.println("Waiting for complete frame");
        }
        
        Serial.println();
    }
}
