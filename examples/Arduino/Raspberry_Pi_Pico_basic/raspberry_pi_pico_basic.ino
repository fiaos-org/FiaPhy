/*Copyright(c) 2025 FIA Operating Systems, All Rights Reserved. Cloud data: https://www.fiaos.org/data*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <FiaPhy.h>

#define PIN_SDA 4
#define PIN_SCL 5

const float FILTER_FACTOR = 0.20f;

const float LATITUDE = 6.9271f;
const float LONGITUDE = 79.8612f;
const float ALTITUDE_M = 5.0f;

const unsigned long SAMPLE_INTERVAL_MS = 1000;
unsigned long last_sample_time = 0;

Adafruit_BME280 bme_ref;
Adafruit_BME280 bme_flux;
FiaPhy::DTDSS dtdss;

struct FilteredData {
    float temp;
    float hum;
    float press;
};

FilteredData data_ref = {0, 0, 0};
FilteredData data_flux = {0, 0, 0};

float apply_filter(float new_val, float old_val) {
    if (old_val == 0.0f) return new_val;
    return (old_val * (1.0f - FILTER_FACTOR)) + (new_val * FILTER_FACTOR);
}

void softStartSystem(float t, float h, float p) {
    Serial.println("Performing Soft-Start Ramp (200 Steps)...");

    for (int i = 1; i <= 200; i++) {
        float factor = (float)i / 200.0f;

        dtdss.feedReferenceTemperature(t * factor, 0);
        dtdss.feedReferenceHumidity(h * factor, 0);
        dtdss.feedReferencePressure(p * factor, 0);

        dtdss.feedFluxTemperature(t * factor, 0);
        dtdss.feedFluxHumidity(h * factor, 0);
        dtdss.feedFluxPressure(p * factor, 0);

        delay(10);
    }
    Serial.println("Soft-Start Complete. Baseline established.");
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("FiaPhy DTDSS - Complete System");
    Serial.println("------------------------------");

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

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

    bme_ref.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X2,
                        Adafruit_BME280::SAMPLING_X16,
                        Adafruit_BME280::SAMPLING_X16,
                        Adafruit_BME280::FILTER_OFF);

    bme_flux.setSampling(Adafruit_BME280::MODE_FORCED,
                         Adafruit_BME280::SAMPLING_X2,
                         Adafruit_BME280::SAMPLING_X16,
                         Adafruit_BME280::SAMPLING_X16,
                         Adafruit_BME280::FILTER_OFF);

    if (dtdss.configure(LATITUDE, LONGITUDE, ALTITUDE_M)) {
        Serial.println("Physics Engine Configured.");
    } else {
        Serial.println("System Configuration Failed");
        while (1) delay(100);
    }

    Serial.print("Acquiring initial readings...");
    bme_ref.takeForcedMeasurement();
    bme_flux.takeForcedMeasurement();
    delay(100);

    data_ref.temp = bme_ref.readTemperature();
    data_ref.hum  = bme_ref.readHumidity();
    data_ref.press = bme_ref.readPressure() / 100.0f;

    data_flux.temp = bme_flux.readTemperature();
    data_flux.hum = bme_flux.readHumidity();
    data_flux.press = bme_flux.readPressure() / 100.0f;
    Serial.println("Done.");

    softStartSystem(data_ref.temp, data_ref.hum, data_ref.press);

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

        bme_ref.takeForcedMeasurement();
        bme_flux.takeForcedMeasurement();
        delay(100);

        float raw_ref_t = bme_ref.readTemperature();
        float raw_ref_h = bme_ref.readHumidity();
        float raw_ref_p = bme_ref.readPressure() / 100.0f;

        float raw_flux_t = bme_flux.readTemperature();
        float raw_flux_h = bme_flux.readHumidity();
        float raw_flux_p = bme_flux.readPressure() / 100.0f;

        data_ref.temp  = apply_filter(raw_ref_t, data_ref.temp);
        data_ref.hum   = apply_filter(raw_ref_h, data_ref.hum);
        data_ref.press = apply_filter(raw_ref_p, data_ref.press);

        data_flux.temp  = apply_filter(raw_flux_t, data_flux.temp);
        data_flux.hum   = apply_filter(raw_flux_h, data_flux.hum);
        data_flux.press = apply_filter(raw_flux_p, data_flux.press);

        dtdss.feedReferenceTemperature(data_ref.temp, 0);
        dtdss.feedReferenceHumidity(data_ref.hum, 0);
        dtdss.feedReferencePressure(data_ref.press, 0);

        dtdss.feedFluxTemperature(data_flux.temp, 0);
        dtdss.feedFluxHumidity(data_flux.hum, 0);
        dtdss.feedFluxPressure(data_flux.press, 0);

        if (dtdss.isFrameReady()) {
            FiaPhy::RadiationResult result = dtdss.compute();

            if (result.valid) {
                Serial.println(">>> DTDSS COMPUTATION SUCCESSFUL <<<");

                Serial.print("Solar Radiation (GHI): ");
                Serial.print(result.ghi_Wm2, 1);
                Serial.println(" W/m2");

                Serial.print("Heat Flux: ");
                Serial.print(result.heat_flux_Wm2, 1);
                Serial.println(" W/m2");

                Serial.print("Temp Differential: ");
                Serial.print(result.temp_differential_C, 3);
                Serial.println(" C");

                Serial.print("Air Density: ");
                Serial.print(result.air_density_kg_m3, 4);
                Serial.println(" kg/m3");

                Serial.print("Cloud Proxy: ");
                Serial.print(result.cloud_proxy * 100.0f, 1);
                Serial.println(" %");

                Serial.println();
            } else {
                Serial.println("[WARN] Computation returned invalid result");
            }
        }
    }
}
