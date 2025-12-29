#ifndef FIAPHY_TYPES_H
#define FIAPHY_TYPES_H

#include <stdint.h>

namespace FiaPhy {

struct SensorReading {
    float tempc;
    float humidity;
    float pressure;
    uint8_t sensorid;
    uint32_t timems;
    
    bool isValid() const {
         return tempc > -100.0f && tempc < 100.0f &&
             humidity >= 0.0f && humidity <= 100.0f &&
             pressure > 300.0f && pressure < 1200.0f;
    }
};


struct SensorFrame {
    float tempc;
    float humidity;
    float pressure;
    uint8_t sensorid;
    bool complete;

    bool hastemp;
    bool hashumidity;
    bool haspressure;
    
    SensorFrame() : tempc(0), humidity(0), pressure(0),
                    sensorid(0), complete(false), hastemp(false),
                    hashumidity(false), haspressure(false) {}
    
    void reset() {
        tempc = 0;
        humidity = 0;
        pressure = 0;
        complete = false;
        hastemp = false;
        hashumidity = false;
        haspressure = false;
    }
    
    void checkCompletion() {
        complete = hastemp && hashumidity && haspressure;
    }
};


struct DifferentialFrame {
    SensorFrame ref;
    SensorFrame flux;
    bool valid;
    
    DifferentialFrame( ) : valid(false) {}
};


struct AtmosphericState {
    float airdensity;
    float vaporpressure;
    float saturationpressure;
    float enthalpy;
    float mixratio;
    float tempk;
};


struct SolarFlux {
    float irradiance;
    float heatflux;
    float solairexcess;
    float cloudfraction;
    float confidence;

    int dayofyear;
    float hourangle;
    float zenithangle;
    float elevationangle;
    float azimuthangle;
    bool sunup;
    float clearskyghi;
    
    SolarFlux() : irradiance(0), heatflux(0),
                  solairexcess(0), cloudfraction(0), confidence(0),
                  dayofyear(0), hourangle(0), zenithangle(0),
                  elevationangle(0), azimuthangle(0), sunup(false),
                  clearskyghi(0) {}
};


struct INRState {
    float filtered;
    float prevfiltered;
    float derivative;
    float projected;
    float alpha;
    uint8_t ringindex;
    static constexpr uint8_t ringsize = 10;
    float ring[ringsize];
    
    INRState() : filtered(0), prevfiltered(0), derivative(0),
                 projected(0), alpha(0.2f), ringindex(0) {
        for (uint8_t i = 0; i < ringsize; i++) ring[i] = 0;
    }
};


struct CalibrationParams {
    float thermaltime;
    float solarabsorb;
    float convectivearea;
    float heatcapacity;
    float selfheatoffset;

    CalibrationParams() :
        thermaltime(30.0f),
        solarabsorb(0.90f),
        convectivearea(6.25e-6f),
        heatcapacity(0.5f),
        selfheatoffset(0.8f) {}
};


enum class ErrorCode : uint8_t {
    OK = 0,
    SENSOR_ASYMMETRY,
    INCOMPLETE_TRIPLET,
    VALUE_OUT_OF_RANGE,
    UNREALISTIC_JUMP,
    INSUFFICIENT_SENSORS,
    BUFFER_OVERFLOW,
    CALIBRATION_FAILED,
    INVALID_STATE
};


struct ValidationResult {
    bool valid;
    ErrorCode errorcode;
    const char* message;
    uint8_t faultysensor;
    
    ValidationResult() : valid(true), errorcode(ErrorCode::OK), message(""), faultysensor(0) {}
    
    bool isOk() const { return valid && errorcode == ErrorCode::OK; }
};


enum class SensorType : uint8_t {
    REFERENCE = 0,
    FLUX = 1
};


struct RadiationResult {
    bool valid;
    float ghi;
    float heatflux;
    float tempdelta;
    float tempchange;
    float solairexcess;
    float baselineghi;
    float cloudproxy;
    float airdensity;
    float vaporpressure;
    float confidence;
    uint32_t timems;
    
    RadiationResult() : valid(false), ghi(0), heatflux(0),
                        tempdelta(0), tempchange(0), solairexcess(0),
                        baselineghi(0), cloudproxy(0), airdensity(0),
                        vaporpressure(0), confidence(0.0f), timems(0) {}
    
    static RadiationResult   invalid() {
        RadiationResult r;
        r.valid = false;
        return r;
    }
};


struct SystemStatus {
    bool configured;
    uint32_t framesdone;
    uint8_t sensorrefcount;
    uint8_t sensorfluxcount;
    
    SystemStatus() : configured(false), framesdone(0),
                     sensorrefcount(0), sensorfluxcount(0) {}
};


struct Configuration {
    float latitude;
    float longitude;
    float altitude;
    
    Configuration() : latitude(0), longitude(0), altitude(0) {}
};


}

#endif
