#include <Wire.h>
#include <Adafruit_BME280.h>
#include <FiaPhy.h>

#define NUM_SENSOR_PAIRS 2
#define ENABLE_SD_LOGGING false
#define ENABLE_WEB_DASH false

Adafruit_BME280 refsensors[NUM_SENSOR_PAIRS];
Adafruit_BME280 fluxsensors[NUM_SENSOR_PAIRS];

const uint8_t refaddresses[8] = {0x76, 0x77, 0x76, 0x77, 0x76, 0x77, 0x76, 0x77};
const uint8_t fluxaddresses[8] = {0x76, 0x77, 0x76, 0x77, 0x76, 0x77, 0x76, 0x77};
FiaPhy::DTDSS dtdss;

struct Stats {
    float ghimean;
    float ghisd;
    uint32_t validframes;
    uint32_t rejectedframes;
    unsigned long uptimems;
} stats;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("FiaPhy™ by fiaos");
    Serial.println("example run: https://fiaos.org/fiaphy/download");
    Serial.println("-------------------------");
    Serial.print("Sensor Pairs:");
    Serial.println(NUM_SENSOR_PAIRS);
    Serial.println();

    Wire.begin();
    Wire.setClock(400000);

    Serial.println("bringing sensors online...");
    for (uint8_t i = 0; i < NUM_SENSOR_PAIRS; i++) {
        if (!initSensorPair(i)) {
            Serial.print("sensor pair ");
            Serial.print(i);
            Serial.println(" failed to start");
            while (1) delay(100);
        }
        Serial.print("sensor pair ");
        Serial.print(i);
        Serial.println(" ready");
    }
    
    dtdss.configure(6.9271, 79.8612, 5.0);

    Serial.println();
    Serial.println("starting auto tune...");
    Serial.println("(this can take around a minute)");
    autoCalibrate();
    
    Serial.println();
    Serial.println("system ready.");
    Serial.println("-----------------");
    Serial.println();
    
    stats = {0, 0, 0, 0, 0};
}

void loop() {
    static unsigned long lastsample = 0;
    const unsigned long samplegap = 1000;
    
    if (millis() - lastsample >= samplegap) {
        lastsample = millis();
        stats.uptimems = millis();
        
        for (uint8_t i = 0; i < NUM_SENSOR_PAIRS; i++) {
            readAndFeedSensorPair(i);
        }
        
        if (dtdss.isFrameReady()) {
            processFrame();
        }
        
        static unsigned long laststats = 0;
        if (millis() - laststats >= 10000) {
            laststats = millis();
            printStatistics();
        }
    }
}

bool initSensorPair(uint8_t pairid) {
    if (!refsensors[pairid].begin(refaddresses[pairid])) {
        return false;
    }
    
    if (!fluxsensors[pairid].begin(fluxaddresses[pairid])) {
        return false;
    }
    
    refsensors[pairid].setSampling(
        Adafruit_BME280::MODE_FORCED,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_OFF
    );
    
    fluxsensors[pairid].setSampling(
        Adafruit_BME280::MODE_FORCED,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_OFF
    );
    
    return true;
}

void readAndFeedSensorPair(uint8_t pairid) {
    refsensors[pairid].takeForcedMeasurement();
    fluxsensors[pairid].takeForcedMeasurement();
    delay(10);
    
    float reftemp = refsensors[pairid].readTemperature();
    float refhum = refsensors[pairid].readHumidity();
    float refpress = refsensors[pairid].readPressure() / 100.0f;
    
    float fluxtemp = fluxsensors[pairid].readTemperature();
    float fluxhum = fluxsensors[pairid].readHumidity();
    float fluxpress = fluxsensors[pairid].readPressure() / 100.0f;
    
    dtdss.feedReferenceTemperature(reftemp, pairid);
    dtdss.feedReferenceHumidity(refhum, pairid);
    dtdss.feedReferencePressure(refpress, pairid);
    
    dtdss.feedFluxTemperature(fluxtemp, pairid);
    dtdss.feedFluxHumidity(fluxhum, pairid);
    dtdss.feedFluxPressure(fluxpress, pairid);
}

void processFrame() {
    FiaPhy::RadiationResult result = dtdss.compute();
    
    if (result.valid) {
        stats.validframes++;
        
        float delta = result.ghi - stats.ghimean;
        stats.ghimean += delta / stats.validframes;
        float delta2 = result.ghi - stats.ghimean;
        stats.ghisd += delta * delta2;
        
        Serial.print("[");
        Serial.print(millis() / 1000);
        Serial.print("s] ghi: ");
        Serial.print(result.ghi, 1);
        Serial.print(" W/m² | cloud: ");
        Serial.print(result.cloudproxy * 100, 0);
        Serial.print("% | dt: ");
        Serial.print(result.tempdelta, 2);
        Serial.println("°C");
        
        if (ENABLE_SD_LOGGING) {
            logToSD(result);
        }
        
    } else {
        stats.rejectedframes++;
        Serial.println("[!] frame validation failed");
    }
}

void autoCalibrate() {
    Serial.println("watching cooldown curve...");
    Serial.println("   (for best result, shade sensors then wait)");

    delay(5000);
    Serial.println("calibration done (defaults used)");
}

void printStatistics() {
    Serial.println();
    Serial.println("sys stats");
    Serial.print("uptime: ");
    Serial.print(stats.uptimems / 1000);
    Serial.println(" seconds");
    Serial.print("valid frames: ");
    Serial.println(stats.validframes);
    Serial.print("rejected frames: ");
    Serial.println(stats.rejectedframes);
    
    if (stats.validframes > 0) {
        Serial.print("mean ghi: ");
        Serial.print(stats.ghimean, 1);
        Serial.println(" W/m²");
        
        float stddev = sqrtf(stats.ghisd / stats.validframes);
        Serial.print("std dev: ");
        Serial.print(stddev, 1);
        Serial.println(" W/m²");
        
        float successrate = 100.0f * stats.validframes /
                            (stats.validframes + stats.rejectedframes);
        Serial.print("success rate: ");
        Serial.print(successrate, 1);
        Serial.println("%");
    }
    
    Serial.println("------------");
    Serial.println();
}

void logToSD(const FiaPhy::RadiationResult& result) {

}

