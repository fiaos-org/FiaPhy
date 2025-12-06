/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

// FiaPhy Arduino Example - BME280 Dual Sensor
// Reference sensor at 0x76 (ventilated), Flux sensor at 0x77 (sealed black-body)

#include <FiaPhy.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme_ref;
Adafruit_BME280 bme_flux;

FiaPhy::SensorHub hub;
FiaPhy::PhysicsEngine engine;

const uint32_t SAMPLE_INTERVAL_MS = 1000;
uint32_t last_sample_time = 0;

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    
    Serial.println("FiaPhy Solar Radiation Monitor");
    Serial.println();
    
    Wire.begin();
    
    if(!bme_ref.begin(0x76)) {
        Serial.println("ERROR: Reference BME280 not found at 0x76");
        while(1) delay(1000);
    }
    
    if(!bme_flux.begin(0x77)) {
        Serial.println("ERROR: Flux BME280 not found at 0x77");
        while(1) delay(1000);
    }
    
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
    
    FiaPhy::initialize();
    
    if(!hub.begin(2)) {
        Serial.println("ERROR: SensorHub initialization failed");
        while(1) delay(1000);
    }
    
    FiaPhy::CalibrationParams calib;
    calib.thermal_time_constant_s = 25.0f;
    calib.solar_absorptivity = 0.92f;
    engine.setCalibration(calib);
    
    Serial.println("Initialization complete");
    Serial.println();
    
    last_sample_time = millis();
}

void loop() {
    uint32_t current_time = millis();
    
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
        
        FiaPhy::ValidationResult result;
        
        result = hub.feedTemperature(temp_ref, 0);
        if(!result.isOk()) printError(result);
        
        result = hub.feedHumidity(humid_ref, 0);
        if(!result.isOk()) printError(result);
        
        result = hub.feedPressure(press_ref, 0);
        if(!result.isOk()) printError(result);
        
        result = hub.feedTemperature(temp_flux, 1);
        if(!result.isOk()) printError(result);
        
        result = hub.feedHumidity(humid_flux, 1);
        if(!result.isOk()) printError(result);
        
        result = hub.feedPressure(press_flux, 1);
        if(!result.isOk()) printError(result);
        
        if(hub.isFrameReady()) {
            const FiaPhy::SensorFrame* frames = hub.getFrames();
            FiaPhy::SolarFlux solar = engine.compute(frames, hub.getSensorCount());
            printResults(frames, solar);
            hub.acknowledgeFrame();
        }
    }
}

void printError(const FiaPhy::ValidationResult& result) {
    Serial.print("VALIDATION ERROR: ");
    Serial.print(result.message);
    Serial.print(" (Sensor ");
    Serial.print(result.faulty_sensor_id);
    Serial.println(")");
}

void printResults(const FiaPhy::SensorFrame* frames, const FiaPhy::SolarFlux& solar) {
    Serial.println("------------------------------------------------");
    Serial.print("Frame #");
    Serial.println(hub.getFrameCounter());
    Serial.println();
    
    Serial.println("Reference Node (Ventilated):");
    Serial.print("  T: "); Serial.print(frames[0].temperature_C, 2); Serial.println(" C");
    Serial.print("  H: "); Serial.print(frames[0].humidity_RH, 1); Serial.println(" %");
    Serial.print("  P: "); Serial.print(frames[0].pressure_hPa, 2); Serial.println(" hPa");
    Serial.println();
    
    Serial.println("Flux Node (Black-Body):");
    Serial.print("  T: "); Serial.print(frames[1].temperature_C, 2); Serial.println(" C");
    Serial.print("  H: "); Serial.print(frames[1].humidity_RH, 1); Serial.println(" %");
    Serial.print("  P: "); Serial.print(frames[1].pressure_hPa, 2); Serial.println(" hPa");
    Serial.println();
    
    float temp_diff = frames[1].temperature_C - frames[0].temperature_C;
    Serial.print("Temperature Differential: "); 
    Serial.print(temp_diff, 3); 
    Serial.println(" C");
    Serial.println();
    
    Serial.println("SOLAR RADIATION");
    Serial.print("  GHI: "); 
    Serial.print(solar.irradiance_Wm2, 1); 
    Serial.println(" W/m2");
    
    Serial.print("  Heat Flux: "); 
    Serial.print(solar.heat_flux_Wm2, 1); 
    Serial.println(" W/m2");
    
    Serial.print("  Cloud Fraction: "); 
    Serial.print(solar.cloud_fraction * 100.0f, 1); 
    Serial.println(" %");
    
    Serial.print("  Confidence: "); 
    Serial.print(solar.confidence * 100.0f, 1); 
    Serial.println(" %");
    
    Serial.println("------------------------------------------------");
    Serial.println();
}
