/*Copyright(c) 2025 FIA Operating Systems, All Rights Reserved. Cloud data: https://www.fiaos.org/data*/

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
    INRFilter inr_ref;
    INRFilter inr_flux;
    Configuration config;

    bool configured;
    uint32_t frame_count;

public:
    DTDSS() : configured(false), frame_count(0) {
        Logger::init();
        Logger::info("FiaPhy DTDSS v1.0.0 initialized");
    }

    bool configure(float latitude, float longitude, float altitude_m = 0) {
        config.latitude = latitude;
        config.longitude = longitude;
        config.altitude_m = altitude_m;

        engine.setLocation(latitude, longitude);

        configured = true;
        Logger::info("System configured for location");
        return true;
    }

    void feedReferenceTemperature(float temp_C, uint8_t sensor_id = 0) {
        hub.feedTemperature(SensorType::REFERENCE, sensor_id, temp_C);
    }

    void feedReferenceHumidity(float humidity_percent, uint8_t sensor_id = 0) {
        hub.feedHumidity(SensorType::REFERENCE, sensor_id, humidity_percent);
    }

    void feedReferencePressure(float pressure_hPa, uint8_t sensor_id = 0) {
        hub.feedPressure(SensorType::REFERENCE, sensor_id, pressure_hPa);
    }

    void feedFluxTemperature(float temp_C, uint8_t sensor_id = 0) {
        hub.feedTemperature(SensorType::FLUX, sensor_id, temp_C);
    }

    void feedFluxHumidity(float humidity_percent, uint8_t sensor_id = 0) {
        hub.feedHumidity(SensorType::FLUX, sensor_id, humidity_percent);
    }

    void feedFluxPressure(float pressure_hPa, uint8_t sensor_id = 0) {
        hub.feedPressure(SensorType::FLUX, sensor_id, pressure_hPa);
    }

    void feedReferenceData(float temp_C, float humidity_percent, float pressure_hPa, uint8_t sensor_id = 0) {
        feedReferenceTemperature(temp_C, sensor_id);
        feedReferenceHumidity(humidity_percent, sensor_id);
        feedReferencePressure(pressure_hPa, sensor_id);
    }

    void feedFluxData(float temp_C, float humidity_percent, float pressure_hPa, uint8_t sensor_id = 0) {
        feedFluxTemperature(temp_C, sensor_id);
        feedFluxHumidity(humidity_percent, sensor_id);
        feedFluxPressure(pressure_hPa, sensor_id);
    }

    bool isFrameReady() {
        return hub.isFrameReady();
    }

    RadiationResult compute() {
        if (!configured) {
            Logger::error("System not configured. Call configure() first.");
            return RadiationResult::invalid();
        }
        
        if (!hub.isFrameReady()) {
            Logger::warn("Frame not ready. Check sensor data completeness.");
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
            if (validation.error_code == ErrorCode::SENSOR_ASYMMETRY) {
                Logger::error("Sensor count asymmetry detected");
            } else if (validation.error_code == ErrorCode::UNREALISTIC_JUMP) {
                Logger::error("Unrealistic sensor value jump detected");
            }
            return RadiationResult::invalid();
        }

        RadiationResult result = engine.compute(frame, inr_ref, inr_flux);

        if (result.valid) {
            frame_count++;
            Logger::debug("DTDSS frame processed successfully");
        }

        return result;
    }

    SystemStatus getStatus() const {
        SystemStatus status;
        status.configured = configured;
        status.frames_processed = frame_count;
        status.sensor_ref_count = hub.getSensorCount(SensorType::REFERENCE);
        status.sensor_flux_count = hub.getSensorCount(SensorType::FLUX);
        return status;
    }

    void reset() {
        hub.reset();
        inr_ref.reset();
        inr_flux.reset();
        frame_count = 0;
        Logger::info("System reset complete");
    }
};

}

#endif
