/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

/**
@file AdvancedMultiSensor.ino. @brief Advanced example with multiple sensor pairs and auto-calibration. @copyright Copyright (c) 2025 FiaOS.org **/

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <FiaPhy.h>

//Config
#define NUM_SENSOR_PAIRS 2       // 22 to 8 pairs are supported.
#define ENABLE_SD_LOGGING false  // please set this true if a SD card present
#define ENABLE_WEB_DASH false    // Set true for ESP32 web interface

// Sensor array
Adafruit_BME280 ref_sensors[NUM_SENSOR_PAIRS];
Adafruit_BME280 flux_sensors[NUM_SENSOR_PAIRS];

// I2C addresses for BME280 (up to 8 pairs on multiplexed bus)
const uint8_t ref_addresses[8] = {0x76, 0x77, 0x76, 0x77, 0x76, 0x77, 0x76, 0x77};
const uint8_t flux_addresses[8] = {0x76, 0x77, 0x76, 0x77, 0x76, 0x77, 0x76, 0x77};

// FiaPhy system
FiaPhy::DTDSS system;

// Statistics
struct Stats {
    float ghi_mean;
    float ghi_std_dev;
    uint32_t valid_frames;
    uint32_t rejected_frames;
    unsigned long uptime_ms;
} stats;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Copyright (c) FiaOS.org");
    Serial.println("FiaPhy version, Example. https://fiaos.org/fiaphy/download");
    Serial.println("-------------------------");
    Serial.print("Sensor Pairs:");
    Serial.println(NUM_SENSOR_PAIRS);
    Serial.println();

    Wire.begin();
    Wire.setClock(400000);
    
    // init all pairs
    Serial.println("Initializing sensors...");
    for (uint8_t i = 0; i < NUM_SENSOR_PAIRS; i++) {
        if (!initSensorPair(i)) {
            Serial.print("ERROR: Sensor pair ");
            Serial.print(i);
            Serial.println(" initialization failed!");
            while (1) delay(100);
        }
        Serial.print("Sensor pair connected.");
        Serial.print(i);
        Serial.println(" initialized");
    }
    
    // Configure FiaPhy
    system.configure(6.9271, 79.8612, 5.0);
    
    // Optional: Auto-calibrate thermal time constant
    Serial.println();
    Serial.println("Starting auto calibration...");
    Serial.println("(This may take ~60 seconds)");
    autoCalibrate();
    
    Serial.println();
    Serial.println("System is ready.");
    Serial.println("-----------------");
    Serial.println();
    
    stats = {0, 0, 0, 0, 0};
}

void loop() {
    static unsigned long last_sample = 0;
    const unsigned long SAMPLE_INTERVAL = 1000;  // 1 Hz
    
    if (millis() - last_sample >= SAMPLE_INTERVAL) {
        last_sample = millis();
        stats.uptime_ms = millis();
        
        // Read all sensor pairs
        for (uint8_t i = 0; i < NUM_SENSOR_PAIRS; i++) {
            readAndFeedSensorPair(i);
        }
        
        // Process frame if ready
        if (system.isFrameReady()) {
            processFrame();
        }
        
        // Print statistics every 10 seconds
        static unsigned long last_stats = 0;
        if (millis() - last_stats >= 10000) {
            last_stats = millis();
            printStatistics();
        }
    }
}

bool initSensorPair(uint8_t pair_id) {
    // Initialize reference sensor
    if (!ref_sensors[pair_id].begin(ref_addresses[pair_id])) {
        return false;
    }
    
    // Initialize flux sensor (may need I2C multiplexer for >2 pairs)
    if (!flux_sensors[pair_id].begin(flux_addresses[pair_id])) {
        return false;
    }
    
    // Configure for low power weather monitoring
    ref_sensors[pair_id].setSampling(
        Adafruit_BME280::MODE_FORCED,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_OFF
    );
    
    flux_sensors[pair_id].setSampling(
        Adafruit_BME280::MODE_FORCED,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_OFF
    );
    
    return true;
}

void readAndFeedSensorPair(uint8_t pair_id) {
    // Trigger measurements
    ref_sensors[pair_id].takeForcedMeasurement();
    flux_sensors[pair_id].takeForcedMeasurement();
    delay(10);
    
    // Read reference sensor
    float ref_temp = ref_sensors[pair_id].readTemperature();
    float ref_hum = ref_sensors[pair_id].readHumidity();
    float ref_pres = ref_sensors[pair_id].readPressure() / 100.0f;
    
    // Read flux sensor
    float flux_temp = flux_sensors[pair_id].readTemperature();
    float flux_hum = flux_sensors[pair_id].readHumidity();
    float flux_pres = flux_sensors[pair_id].readPressure() / 100.0f;
    
    // Feed to FiaPhy (asynchronously)
    system.feedReferenceTemperature(ref_temp, pair_id);
    system.feedReferenceHumidity(ref_hum, pair_id);
    system.feedReferencePressure(ref_pres, pair_id);
    
    system.feedFluxTemperature(flux_temp, pair_id);
    system.feedFluxHumidity(flux_hum, pair_id);
    system.feedFluxPressure(flux_pres, pair_id);
}

void processFrame() {
    FiaPhy::RadiationResult result = system.compute();
    
    if (result.valid) {
        stats.valid_frames++;
        
        // Update running statistics
        float delta = result.ghi_Wm2 - stats.ghi_mean;
        stats.ghi_mean += delta / stats.valid_frames;
        float delta2 = result.ghi_Wm2 - stats.ghi_mean;
        stats.ghi_std_dev += delta * delta2;
        
        // Print compact result
        Serial.print("[");
        Serial.print(millis() / 1000);
        Serial.print("s] GHI: ");
        Serial.print(result.ghi_Wm2, 1);
        Serial.print(" W/m²  |  Cloud: ");
        Serial.print(result.cloud_proxy * 100, 0);
        Serial.print("%  |  ΔT: ");
        Serial.print(result.temp_differential_C, 2);
        Serial.println("°C");
        
        // Optional: Log to SD card
        if (ENABLE_SD_LOGGING) {
            logToSD(result);
        }
        
    } else {
        stats.rejected_frames++;
        Serial.println("[!] Frame validation failed");
    }
}

void autoCalibrate() {
    // Simplified auto-calibration routine
    // Full implementation would measure step response in controlled conditions
    Serial.println("Monitoring temperature decay...");
    Serial.println("   (For best results, shade sensors then observe cooling)");
     
    delay(5000);  // adjust custom calib. time
    Serial.println("Calibration complete (using defaults)");
}

void printStatistics() {
    Serial.println();
    Serial.println("Sys. Stats");
    Serial.print("Uptime: ");
    Serial.print(stats.uptime_ms / 1000);
    Serial.println(" seconds");
    Serial.print("Valid Frames: ");
    Serial.println(stats.valid_frames);
    Serial.print("Rejected Frames: ");
    Serial.println(stats.rejected_frames);
    
    if (stats.valid_frames > 0) {
        Serial.print("Mean GHI: ");
        Serial.print(stats.ghi_mean, 1);
        Serial.println(" W/m²");
        
        float std_dev = sqrtf(stats.ghi_std_dev / stats.valid_frames);
        Serial.print("Std Dev: ");
        Serial.print(std_dev, 1);
        Serial.println(" W/m²");
        
        float success_rate = 100.0f * stats.valid_frames / 
                            (stats.valid_frames + stats.rejected_frames);
        Serial.print("Success Rate: ");
        Serial.print(success_rate, 1);
        Serial.println("%");
    }
    
    Serial.println("------------");
    Serial.println();
}

void logToSD(const FiaPhy::RadiationResult& result) {
    
    //SD card logging...
}

