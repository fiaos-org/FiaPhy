#include <FiaPhy.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bmeref;
Adafruit_BME280 bmeflux;

FiaPhy::SensorHub hub;
FiaPhy::PhysicsEngine engine;

const uint32_t samplegap = 1000;
uint32_t lastsample = 0;

void setup() {
    Serial.begin(115200);
    while ( !Serial ) delay(10);
    
    Serial.println("FiaPhy™ live sky readout");
    Serial.println();
    
    Wire.begin();
    
    if (!bmeref.begin(0x76)) {
        Serial.println("oops, ref bme280 missing at 0x76");
        while(1) delay(1000);
    }
    
    if (!bmeflux.begin(0x77)) {
        Serial.println("oops, flux bme280 missing at 0x77");
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
    
    if ( !hub.begin(2) ) {
        Serial.println("ERROR: SensorHub initialization failed");
        while(1) delay(1000);
    }
    
    FiaPhy::CalibrationParams tune;
    tune.thermaltime = 25.0f;
    tune.solarabsorb = 0.92f;
    engine.setCalibration(tune);
    
    Serial.println("nice, setup done");
    Serial.println();
    
    lastsample = millis();
}

void loop() {
    uint32_t nowms = millis();
    
    if (nowms - lastsample >= samplegap) {
        lastsample = nowms;
        
        bmeref.takeForcedMeasurement();
        bmeflux.takeForcedMeasurement();
        
        float reftemp = bmeref.readTemperature();
        float refhum = bmeref.readHumidity();
        float refpress = bmeref.readPressure() / 100.0f;
        
        float fluxtemp = bmeflux.readTemperature();
        float fluxhum = bmeflux.readHumidity();
        float fluxpress = bmeflux.readPressure() / 100.0f;
        
        FiaPhy::ValidationResult check;
        
        check = hub.feedTemperature(reftemp, 0);
        if(!check.isOk()) printError(check);
        
        check = hub.feedHumidity(refhum, 0);
        if(!check.isOk()) printError(check);
        
        check = hub.feedPressure(refpress, 0);
        if(!check.isOk()) printError(check);
        
        check = hub.feedTemperature(fluxtemp, 1);
        if(!check.isOk()) printError(check);
        
        check = hub.feedHumidity(fluxhum, 1);
        if(!check.isOk()) printError(check);
        
        check = hub.feedPressure(fluxpress, 1);
        if(!check.isOk()) printError(check);
        
        if(hub.isFrameReady()) {
            const FiaPhy::SensorFrame* frames = hub.getFrames();
            FiaPhy::SolarFlux sky = engine.compute(frames, hub.getSensorCount());
            printResults(frames, sky);
            hub.acknowledgeFrame();
        }
    }
}

void printError(const FiaPhy::ValidationResult& check) {
    Serial.print("data check failed: ");
    Serial.print(check.message);
    Serial.print(" (sensor ");
    Serial.print(check.faultysensor);
    Serial.println(")");
}

void printResults(const FiaPhy::SensorFrame* frames, const FiaPhy::SolarFlux& sky) {
    Serial.println("------------------------------------------------");
    Serial.print("frame #");
    Serial.println(hub.getFrameCounter());
    Serial.println();
    
    Serial.println("ref side:");
    Serial.print("  t: "); Serial.print(frames[0].tempc, 2); Serial.println(" C");
    Serial.print("  h: "); Serial.print(frames[0].humidity, 1); Serial.println(" %");
    Serial.print("  p: "); Serial.print(frames[0].pressure, 2); Serial.println(" hPa");
    Serial.println();
    
    Serial.println("flux side:");
    Serial.print("  t: "); Serial.print(frames[1].tempc, 2); Serial.println(" C");
    Serial.print("  h: "); Serial.print(frames[1].humidity, 1); Serial.println(" %");
    Serial.print("  p: "); Serial.print(frames[1].pressure, 2); Serial.println(" hPa");
    Serial.println();
    
    float tempgap = frames[1].tempc - frames[0].tempc;
    Serial.print("temp gap: ");
    Serial.print(tempgap, 3);
    Serial.println(" C");
    Serial.println();
    
    Serial.println("sky estimate");
    Serial.print("  ghi: ");
    Serial.print(sky.irradiance, 1);
    Serial.println(" W/m2");
    
    Serial.print("  heat flux: ");
    Serial.print(sky.heatflux, 1);
    Serial.println(" W/m2");
    
    Serial.print("  cloud: ");
    Serial.print(sky.cloudfraction * 100.0f, 1);
    Serial.println(" %");
    
    Serial.print("  confidence: ");
    Serial.print(sky.confidence * 100.0f, 1);
    Serial.println(" %");
    
    Serial.println("------------------------------------------------");
    Serial.println();
}
