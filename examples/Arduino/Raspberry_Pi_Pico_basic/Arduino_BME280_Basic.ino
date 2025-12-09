/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

/**
 * @file Pico_BME280_Complete.ino
 * @brief FiaPhy DTDSS for Raspberry Pi Pico
 * @details Complete implementation with Soft-Start, Noise Filtering, and Heat Flux output.
 * * Hardware:
 * - Raspberry Pi Pico
 * - 2x BME280 Sensors
 * - Wiring: SDA -> GP4 (Physical 6), SCL -> GP5 (Physical 7)
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <FiaPhy.h>

// --- CONFIGURATION ---

// PICO PIN DEFINITIONS
#define PIN_SDA 4 // GP4 (Physical Pin 6)
#define PIN_SCL 5 // GP5 (Physical Pin 7)

// FILTER SETTINGS
// 0.05 = Heavy filtering (95% old value, 5% new value)
// This is required to make consumer sensors stable enough for physics calculations.
const float FILTER_FACTOR = 0.20f;

// LOCATION (Sri Lanka)
const float LATITUDE = 6.9271f;
const float LONGITUDE = 79.8612f;
const float ALTITUDE_M = 5.0f;

// TIMING
const unsigned long SAMPLE_INTERVAL_MS = 1000;
unsigned long last_sample_time = 0;

// --- OBJECTS ---
Adafruit_BME280 bme_ref;  // Address 0x76 (SDO -> GND)
Adafruit_BME280 bme_flux; // Address 0x77 (SDO -> VCC)
FiaPhy::DTDSS dtdss;

// --- STORAGE ---
struct FilteredData {
    float temp;
    float hum;
    float press;
};

// Initialize filter memory with zeros
FilteredData data_ref = {0, 0, 0};
FilteredData data_flux = {0, 0, 0};

// --- HELPER FUNCTIONS ---

// 1. Low Pass Filter (The "Flywheel")
float apply_filter(float new_val, float old_val) {
    if (old_val == 0.0f) return new_val; // First run catch
    return (old_val * (1.0f - FILTER_FACTOR)) + (new_val * FILTER_FACTOR);
}

// 2. Soft Start Ramp
// Feeds 200 incremental frames to walk the library memory from 0 to current values.
// This prevents the "Unrealistic Jump" safety check from triggering on boot.
void softStartSystem(float t, float h, float p) {
    Serial.println("Performing Soft-Start Ramp (200 Steps)...");
    
    // We use 200 steps to keep pressure changes small (< 5 hPa per step)
    for (int i = 1; i <= 200; i++) {
        float factor = (float)i / 200.0f;
        
        // Feed interpolated values (ID = 0)
        dtdss.feedReferenceTemperature(t * factor, 0);
        dtdss.feedReferenceHumidity(h * factor, 0);
        dtdss.feedReferencePressure(p * factor, 0);
        
        dtdss.feedFluxTemperature(t * factor, 0);
        dtdss.feedFluxHumidity(h * factor, 0);
        dtdss.feedFluxPressure(p * factor, 0);
        
        // Short delay to allow processing
        delay(10); 
    }
    Serial.println("Soft-Start Complete. Baseline established.");
}

void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for Serial Monitor
    
    Serial.println("FiaPhy DTDSS - Complete System");
    Serial.println("------------------------------");
    
    // 1. I2C Setup for Pico
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    
    // 2. Sensor Initialization
    if (!bme_ref.begin(0x76)) {
        Serial.println("ERROR: Ref Sensor (0x76) not found. Check SDO -> GND");
        while (1) delay(100);
    }
    Serial.println("Reference Sensor OK (0x76)");
    
    if (!bme_flux.begin(0x77)) {
        Serial.println("ERROR: Flux Sensor (0x77) not found. Check SDO -> VCC");
        while (1) delay(100);
    }
    Serial.println("Flux Sensor OK (0x77)");
    
    // 3. Sensor Configuration (High Oversampling for Noise Reduction)
    bme_ref.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X2,   // Temp
                        Adafruit_BME280::SAMPLING_X16,  // Pressure
                        Adafruit_BME280::SAMPLING_X16,  // Humidity
                        Adafruit_BME280::FILTER_OFF);
    
    bme_flux.setSampling(Adafruit_BME280::MODE_FORCED,
                         Adafruit_BME280::SAMPLING_X2,
                         Adafruit_BME280::SAMPLING_X16, 
                         Adafruit_BME280::SAMPLING_X16, 
                         Adafruit_BME280::FILTER_OFF);
    
    // 4. FiaPhy System Configuration
    if (dtdss.configure(LATITUDE, LONGITUDE, ALTITUDE_M)) {
        Serial.println("Physics Engine Configured.");
    } else {
        Serial.println("System Configuration Failed");
        while (1) delay(100);
    }
    
    // 5. Initial Reading & Soft Start
    Serial.print("Acquiring initial readings...");
    bme_ref.takeForcedMeasurement();
    bme_flux.takeForcedMeasurement();
    delay(100);
    
    // Read raw values into our filter structs
    data_ref.temp = bme_ref.readTemperature();
    data_ref.hum = bme_ref.readHumidity();
    data_ref.press = bme_ref.readPressure() / 100.0f;
    
    data_flux.temp = bme_flux.readTemperature();
    data_flux.hum = bme_flux.readHumidity();
    data_flux.press = bme_flux.readPressure() / 100.0f;
    Serial.println("Done.");
    
    // Run the ramp
    softStartSystem(data_ref.temp, data_ref.hum, data_ref.press);
    
    // Clear any "Ready" flags left over from the soft start
    if (dtdss.isFrameReady()) {
        dtdss.compute(); 
    }
    
    Serial.println("System Ready - Starting Main Loop");
    Serial.println();
}

void loop() {
    unsigned long current_time = millis();
    
    if (current_time - last_sample_time >= SAMPLE_INTERVAL_MS) {
        last_sample_time = current_time;
        
        // 1. Trigger Measurements
        bme_ref.takeForcedMeasurement();
        bme_flux.takeForcedMeasurement();
        delay(100); // Wait for X16 oversampling
        
        // 2. Read Raw Data
        float raw_ref_t = bme_ref.readTemperature();
        float raw_ref_h = bme_ref.readHumidity();
        float raw_ref_p = bme_ref.readPressure() / 100.0f;
        
        float raw_flux_t = bme_flux.readTemperature();
        float raw_flux_h = bme_flux.readHumidity();
        float raw_flux_p = bme_flux.readPressure() / 100.0f;
        
        // 3. Apply Software Smoothing (Heavy Filter)
        data_ref.temp  = apply_filter(raw_ref_t, data_ref.temp);
        data_ref.hum   = apply_filter(raw_ref_h, data_ref.hum);
        data_ref.press = apply_filter(raw_ref_p, data_ref.press);

        data_flux.temp  = apply_filter(raw_flux_t, data_flux.temp);
        data_flux.hum   = apply_filter(raw_flux_h, data_flux.hum);
        data_flux.press = apply_filter(raw_flux_p, data_flux.press);

        // 4. Feed Filtered Data to System
        dtdss.feedReferenceTemperature(data_ref.temp, 0);
        dtdss.feedReferenceHumidity(data_ref.hum, 0);
        dtdss.feedReferencePressure(data_ref.press, 0);
        
        dtdss.feedFluxTemperature(data_flux.temp, 0);
        dtdss.feedFluxHumidity(data_flux.hum, 0);
        dtdss.feedFluxPressure(data_flux.press, 0);
        
        // 5. Compute and Display Results
        if (dtdss.isFrameReady()) {
            FiaPhy::RadiationResult result = dtdss.compute();
            
            if (result.valid) {
                Serial.println(">>> DTDSS COMPUTATION SUCCESSFUL <<<");
                
                // GHI (Global Horizontal Irradiance)
                Serial.print("Solar Radiation (GHI): ");
                Serial.print(result.ghi_Wm2, 1);
                Serial.println(" W/m2");
                
                // Heat Flux (Pure thermal component)
                Serial.print("Convective Heat Flux:  ");
                Serial.print(result.heat_flux_Wm2, 1);
                Serial.println(" W/m2");
                
                // Physics Diagnostics
                Serial.print("Temp Delta: ");
                Serial.print(result.temp_differential_C, 3);
                Serial.println(" C");
                
                Serial.print("Cloud Fraction: ");
                Serial.print(result.cloud_proxy * 100.0f, 0);
                Serial.println("%");
                
                Serial.print("Sensor P: ");
                Serial.print(data_ref.press, 2);
                Serial.println(" hPa");
                
                Serial.println("------------------------------------");
            } else {
                // If computation runs but physics rejects the result (e.g. negative values)
                Serial.println("Valid frame, but unstable physics result. stabilizing...");
            }
        } else {
            Serial.println("Buffering data..."); 
        }
        Serial.println();
    }
}