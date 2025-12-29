#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <FiaPhy.h>

#define PIN_SDA 4
#define PIN_SCL 5

const float filterfactor = 0.20f;

const float LATITUDE = 6.9271f;
const float LONGITUDE = 79.8612f;
const float ALTITUDE = 5.0f;

const unsigned long samplegap = 1000;
unsigned long lastsample = 0;

Adafruit_BME280 bmeref;
Adafruit_BME280 bmeflux;
FiaPhy::DTDSS dtdss;

struct FilteredData {
    float temp;
    float hum;
    float press;
};

FilteredData dataref = {0, 0, 0};
FilteredData dataflux = {0, 0, 0};

float applyfilter(float newval, float oldval) {
    if (oldval == 0.0f) return newval;
    return (oldval * (1.0f - filterfactor)) + (newval * filterfactor);
}

void softStartSystem( float t, float h, float p ) {
    Serial.println("doing soft start ramp (200 steps)...");

    for ( int i = 1; i <= 200; i++ ) {
        float factor = (float)i / 200.0f;

        dtdss.feedReferenceTemperature(t * factor, 0);
        dtdss.feedReferenceHumidity(h * factor, 0);
        dtdss.feedReferencePressure(p * factor, 0);

        dtdss.feedFluxTemperature(t * factor, 0);
        dtdss.feedFluxHumidity(h * factor, 0);
        dtdss.feedFluxPressure(p * factor, 0);

        delay(10);
    }
    Serial.println("soft start done, baseline set.");
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("FiaPhy™ dtdss full run");
    Serial.println("------------------------------");

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!bmeref.begin(0x76)) {
        Serial.println("ref sensor missing at 0x76, check wiring");
        while (1) delay(100);
    }
    Serial.println("ref sensor ok (0x76)");

    if (!bmeflux.begin(0x77)) {
        Serial.println("flux sensor missing at 0x77, check wiring");
        while (1) delay(100);
    }
    Serial.println("flux sensor ok (0x77)");

    bmeref.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X2,
                        Adafruit_BME280::SAMPLING_X16,
                        Adafruit_BME280::SAMPLING_X16,
                        Adafruit_BME280::FILTER_OFF);

    bmeflux.setSampling(Adafruit_BME280::MODE_FORCED,
                         Adafruit_BME280::SAMPLING_X2,
                         Adafruit_BME280::SAMPLING_X16,
                         Adafruit_BME280::SAMPLING_X16,
                         Adafruit_BME280::FILTER_OFF);

    if (dtdss.configure(LATITUDE, LONGITUDE, ALTITUDE)) {
        Serial.println("physics engine ready.");
    } else {
        Serial.println("setup failed");
        while (1) delay(100);
    }

    Serial.print("Acquiring initial readings...");
    bmeref.takeForcedMeasurement();
    bmeflux.takeForcedMeasurement();
    delay(100);

    dataref.temp = bmeref.readTemperature();
    dataref.hum  = bmeref.readHumidity();
    dataref.press = bmeref.readPressure() / 100.0f;

    dataflux.temp = bmeflux.readTemperature();
    dataflux.hum = bmeflux.readHumidity();
    dataflux.press = bmeflux.readPressure() / 100.0f;
    Serial.println("Done.");

    softStartSystem(dataref.temp, dataref.hum, dataref.press);

    if (dtdss.isFrameReady()) {
        dtdss.compute();
    }

    Serial.println("ready, entering main loop");
    Serial.println();
}

void loop() {
    unsigned long nowms = millis();

    if (nowms - lastsample >= samplegap) {
        lastsample = nowms;

        bmeref.takeForcedMeasurement();
        bmeflux.takeForcedMeasurement();
        delay(100);

        float rawreft = bmeref.readTemperature();
        float rawrefh = bmeref.readHumidity();
        float rawrefp = bmeref.readPressure() / 100.0f;

        float rawfluxt = bmeflux.readTemperature();
        float rawfluxh = bmeflux.readHumidity();
        float rawfluxp = bmeflux.readPressure() / 100.0f;

        dataref.temp  = applyfilter(rawreft, dataref.temp);
        dataref.hum   = applyfilter(rawrefh, dataref.hum);
        dataref.press = applyfilter(rawrefp, dataref.press);

        dataflux.temp  = applyfilter(rawfluxt, dataflux.temp);
        dataflux.hum   = applyfilter(rawfluxh, dataflux.hum);
        dataflux.press = applyfilter(rawfluxp, dataflux.press);

        dtdss.feedReferenceTemperature(dataref.temp, 0);
        dtdss.feedReferenceHumidity(dataref.hum, 0);
        dtdss.feedReferencePressure(dataref.press, 0);

        dtdss.feedFluxTemperature(dataflux.temp, 0);
        dtdss.feedFluxHumidity(dataflux.hum, 0);
        dtdss.feedFluxPressure(dataflux.press, 0);

        if (dtdss.isFrameReady()) {
            FiaPhy::RadiationResult result = dtdss.compute();

            if (result.valid) {
                Serial.println(">>> compute pass <<<");

                Serial.print("ghi: ");
                Serial.print(result.ghi, 1);
                Serial.println(" W/m2");

                Serial.print("heat flux: ");
                Serial.print(result.heatflux, 1);
                Serial.println(" W/m2");

                Serial.print("temp gap: ");
                Serial.print(result.tempdelta, 3);
                Serial.println(" C");

                Serial.print("air density: ");
                Serial.print(result.airdensity, 4);
                Serial.println(" kg/m3");

                Serial.print("cloud proxy: ");
                Serial.print(result.cloudproxy * 100.0f, 1);
                Serial.println(" %");

                Serial.println();
            } else {
                Serial.println("[warn] compute returned invalid");
            }
        }
    }
}
