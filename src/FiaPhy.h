#ifndef FIAPHY_H
#define FIAPHY_H

#include "core/Types.h"
#include "core/Constants.h"
#include "core/SensorHub.h"
#include "core/PhysicsEngine.h"
#include "physics/Thermodynamics.h"
#include "physics/SolarGeometry.h"
#include "filters/INR.h"
#include "platform/Logger.h"

namespace FiaPhy {


class DTDSS {
private:
    SensorHub hub;
    PhysicsEngine engine;
    INRFilter inrref;
    INRFilter inrflux;
    Configuration config;

    bool configured;
    uint32_t framecount;

public:
    DTDSS() : configured(false), framecount(0) {
        Logger::init();
        Logger::info("FiaPhy DTDSS v1.0.0 initialized");
    }

    bool configure(float latitude, float longitude, float altitude = 0) {
        config.latitude = latitude;
        config.longitude = longitude;
        config.altitude = altitude;

        engine.setLocation(latitude, longitude);

        configured = true;
        Logger::info("System configured for location");
        return true;
    }

    void feedReferenceTemperature(float tempc, uint8_t sensorid = 0) {
        hub.feedTemperature(SensorType::REFERENCE, sensorid, tempc);
    }

    void feedReferenceHumidity(float humidity, uint8_t sensorid = 0) {
        hub.feedHumidity(SensorType::REFERENCE, sensorid, humidity);
    }

    void feedReferencePressure(float pressure, uint8_t sensorid = 0) {
        hub.feedPressure(SensorType::REFERENCE, sensorid, pressure);
    }

    void feedFluxTemperature(float tempc, uint8_t sensorid = 0) {
        hub.feedTemperature(SensorType::FLUX, sensorid, tempc);
    }

    void feedFluxHumidity(float humidity, uint8_t sensorid = 0) {
        hub.feedHumidity(SensorType::FLUX, sensorid, humidity);
    }

    void feedFluxPressure(float pressure, uint8_t sensorid = 0) {
        hub.feedPressure(SensorType::FLUX, sensorid, pressure);
    }

    void feedReferenceData(float tempc, float humidity, float pressure, uint8_t sensorid = 0) {
        feedReferenceTemperature(tempc, sensorid);
        feedReferenceHumidity(humidity, sensorid);
        feedReferencePressure(pressure, sensorid);
    }

    void feedFluxData(float tempc, float humidity, float pressure, uint8_t sensorid = 0) {
        feedFluxTemperature(tempc, sensorid);
        feedFluxHumidity(humidity, sensorid);
        feedFluxPressure(pressure, sensorid);
    }

    bool isFrameReady( ) {
        return hub.isFrameReady();
    }

    RadiationResult  compute() {
        if (!configured) {
            Logger::error("Setup missing, run configure() first.");
            return RadiationResult::invalid();
        }
        
        if (!hub.isFrameReady()) {
            Logger::warn("Frame still building, waiting for full sensor set.");
            return RadiationResult::invalid();
        }

        DifferentialFrame frame = hub.getDifferentialFrame();

        if (!frame.valid) {
            Logger::error("Failed to obtain valid differential frame");
            return RadiationResult::invalid();
        }

        ValidationResult validation = hub.validateFrame(frame.ref);
        if (!validation.valid) {
            Logger::error("Frame validation failed");
            if (validation.errorcode == ErrorCode::SENSOR_ASYMMETRY) {
                Logger::error("Sensor pair count mismatch detected");
            } else if (validation.errorcode == ErrorCode::UNREALISTIC_JUMP) {
                Logger::error("Sensor jump looked too sharp");
            }
            return RadiationResult::invalid();
        }

        RadiationResult result = engine.compute(frame, inrref, inrflux);

        if (result.valid) {
            framecount++;
            Logger::debug("DTDSS frame processed successfully");
        }

        return result;
    }

    SystemStatus getStatus()  const {
        SystemStatus status;
        status.configured = configured;
        status.framesdone = framecount;
        status.sensorrefcount = hub.getSensorCount(SensorType::REFERENCE);
        status.sensorfluxcount = hub.getSensorCount(SensorType::FLUX);
        return status;
    }

    void reset() {
        hub.reset();
        inrref.reset();
        inrflux.reset();
        framecount = 0;
        Logger::info("System reset complete");
    }
};

}

#endif
