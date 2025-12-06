/**
 * @file FiaPhy.h
 * @brief Main entry point for the FiaPhy DTDSS Library
 * @version 1.0.0
 * @date 2025-12-06
 * 
 * FiaPhy: Differential Temporal Derivative Soft-Sensing Library
 * Transforms commodity environmental sensors into capability-dense radiometers
 * 
 * Based on the research paper:
 * "Temporal Derivative Soft-Sensing and Reconstructing Solar Radiation 
 * and Heat Flux from Common Environmental Sensors"
 * by Neksha V. DeSilva, FiaOS.org
 * 
 * @copyright Copyright (c) 2025 FiaOS.org
 * @license MIT License
 * 
 * Hardware Support:
 * - Arduino (8-bit AVR, ARM Cortex-M0/M3/M4)
 * - ESP32/ESP8266
 * - Raspberry Pi (Linux ARM)
 * - PlatformIO compatible boards
 * 
 * Key Features:
 * - Dual-sensor differential architecture
 * - Inertial Noise Reduction (INR) filtering
 * - Altitude-agnostic (sea level to 5000m)
 * - Header-only implementation (no linking required)
 * - Minimal memory footprint (<60 bytes RAM)
 * - Fixed-point arithmetic for 8-bit MCUs
 */

#ifndef FIAPHY_H
#define FIAPHY_H

// Core system headers
#include "core/Types.h"
#include "core/Constants.h"
#include "core/SensorHub.h"
#include "core/PhysicsEngine.h"

// Physics modules
#include "physics/Thermodynamics.h"
#include "physics/SolarGeometry.h"

// Signal processing
#include "filters/INR.h"

// Platform abstraction
#include "platform/Logger.h"

/**
 * @namespace FiaPhy
 * @brief Main namespace for all FiaPhy library components
 */
namespace FiaPhy {

/**
 * @class DTDSS
 * @brief High-level interface for Differential Temporal Derivative Soft-Sensing
 * 
 * This class orchestrates the entire sensing pipeline:
 * 1. Asynchronous sensor data ingestion via SensorHub
 * 2. Differential temperature analysis
 * 3. INR filtering for noise reduction
 * 4. Physics-based radiation reconstruction
 * 
 * @example Basic usage:
 * @code
 * FiaPhy::DTDSS system;
 * system.configure(latitude, longitude, altitude);
 * 
 * // Feed sensor data asynchronously
 * system.feedReferenceData(temp, humidity, pressure);
 * system.feedFluxData(temp, humidity, pressure);
 * 
 * // Check if complete frame is ready
 * if (system.isFrameReady()) {
 *     FiaPhy::RadiationResult result = system.compute();
 *     if (result.valid) {
 *         Serial.print("GHI: ");
 *         Serial.println(result.ghi_Wm2);
 *     }
 * }
 * @endcode
 */
class DTDSS {
private:
    SensorHub hub;
    PhysicsEngine engine;
    INRFilter inr_ref;
    INRFilter inr_flux;
    Configuration config;
    
    // State tracking
    bool configured;
    uint32_t frame_count;
    
public:
    /**
     * @brief Construct a new DTDSS system
     */
    DTDSS() : configured(false), frame_count(0) {
        Logger::init();
        Logger::info("FiaPhy DTDSS v1.0.0 initialized");
    }
    
    /**
     * @brief Configure the system for a specific deployment location
     * 
     * @param latitude Latitude in decimal degrees (-90 to +90)
     * @param longitude Longitude in decimal degrees (-180 to +180)
     * @param altitude_m Altitude above sea level in meters (optional, derived from pressure if 0)
     * @return true if configuration successful
     */
    bool configure(float latitude, float longitude, float altitude_m = 0) {
        config.latitude = latitude;
        config.longitude = longitude;
        config.altitude_m = altitude_m;
        
        // Initialize physics engine with location
        engine.setLocation(latitude, longitude);
        
        configured = true;
        Logger::info("System configured for location");
        return true;
    }
    
    /**
     * @brief Feed temperature data from Reference sensor (ventilated, shielded)
     * 
     * This method accepts sensor data asynchronously. The system will buffer
     * data internally until a complete frame (T+H+P triplet) is collected.
     * 
     * @param temp_C Temperature in Celsius
     * @param sensor_id Sensor identifier (default 0)
     */
    void feedReferenceTemperature(float temp_C, uint8_t sensor_id = 0) {
        hub.feedTemperature(SensorType::REFERENCE, sensor_id, temp_C);
    }
    
    /**
     * @brief Feed humidity data from Reference sensor
     * 
     * @param humidity_percent Relative humidity (0-100%)
     * @param sensor_id Sensor identifier (default 0)
     */
    void feedReferenceHumidity(float humidity_percent, uint8_t sensor_id = 0) {
        hub.feedHumidity(SensorType::REFERENCE, sensor_id, humidity_percent);
    }
    
    /**
     * @brief Feed pressure data from Reference sensor
     * 
     * @param pressure_hPa Atmospheric pressure in hectopascals (hPa)
     * @param sensor_id Sensor identifier (default 0)
     */
    void feedReferencePressure(float pressure_hPa, uint8_t sensor_id = 0) {
        hub.feedPressure(SensorType::REFERENCE, sensor_id, pressure_hPa);
    }
    
    /**
     * @brief Feed temperature data from Flux sensor (black-body, absorptive)
     * 
     * @param temp_C Temperature in Celsius
     * @param sensor_id Sensor identifier (default 0)
     */
    void feedFluxTemperature(float temp_C, uint8_t sensor_id = 0) {
        hub.feedTemperature(SensorType::FLUX, sensor_id, temp_C);
    }
    
    /**
     * @brief Feed humidity data from Flux sensor
     * 
     * @param humidity_percent Relative humidity (0-100%)
     * @param sensor_id Sensor identifier (default 0)
     */
    void feedFluxHumidity(float humidity_percent, uint8_t sensor_id = 0) {
        hub.feedHumidity(SensorType::FLUX, sensor_id, humidity_percent);
    }
    
    /**
     * @brief Feed pressure data from Flux sensor
     * 
     * @param pressure_hPa Atmospheric pressure in hectopascals (hPa)
     * @param sensor_id Sensor identifier (default 0)
     */
    void feedFluxPressure(float pressure_hPa, uint8_t sensor_id = 0) {
        hub.feedPressure(SensorType::FLUX, sensor_id, pressure_hPa);
    }
    
    /**
     * @brief Feed complete sensor reading from Reference sensor
     * 
     * Convenience method for submitting all three parameters at once
     * 
     * @param temp_C Temperature in Celsius
     * @param humidity_percent Relative humidity (0-100%)
     * @param pressure_hPa Atmospheric pressure in hectopascals
     * @param sensor_id Sensor identifier (default 0)
     */
    void feedReferenceData(float temp_C, float humidity_percent, float pressure_hPa, uint8_t sensor_id = 0) {
        feedReferenceTemperature(temp_C, sensor_id);
        feedReferenceHumidity(humidity_percent, sensor_id);
        feedReferencePressure(pressure_hPa, sensor_id);
    }
    
    /**
     * @brief Feed complete sensor reading from Flux sensor
     * 
     * @param temp_C Temperature in Celsius
     * @param humidity_percent Relative humidity (0-100%)
     * @param pressure_hPa Atmospheric pressure in hectopascals
     * @param sensor_id Sensor identifier (default 0)
     */
    void feedFluxData(float temp_C, float humidity_percent, float pressure_hPa, uint8_t sensor_id = 0) {
        feedFluxTemperature(temp_C, sensor_id);
        feedFluxHumidity(humidity_percent, sensor_id);
        feedFluxPressure(pressure_hPa, sensor_id);
    }
    
    /**
     * @brief Check if a complete frame is ready for processing
     * 
     * A frame is ready when:
     * - Both Reference and Flux sensors have complete T+H+P triplets
     * - Sensor counts are symmetric (equal number of each type)
     * - No unrealistic jumps detected
     * 
     * @return true if frame is ready for computation
     */
    bool isFrameReady() {
        return hub.isFrameReady();
    }
    
    /**
     * @brief Compute solar radiation and heat flux from buffered sensor data
     * 
     * This method performs the complete DTDSS pipeline:
     * 1. Validates frame integrity (symmetry, realistic values)
     * 2. Applies INR filtering to temperature derivatives
     * 3. Calculates air density and thermodynamic properties
     * 4. Executes Reference Path (Kasten-Czeplak baseline)
     * 5. Executes Reactive Path (differential + INR)
     * 6. Returns fused radiation estimate
     * 
     * @return RadiationResult containing GHI, heat flux, and validation status
     */
    RadiationResult compute() {
        if (!configured) {
            Logger::error("System not configured. Call configure() first.");
            return RadiationResult::invalid();
        }
        
        if (!hub.isFrameReady()) {
            Logger::warning("Frame not ready. Check sensor data completeness.");
            return RadiationResult::invalid();
        }
        
        // Retrieve validated frame
        SensorFrame frame = hub.getFrame();
        
        // Validate frame integrity
        ValidationResult validation = hub.validateFrame(frame);
        if (!validation.valid) {
            Logger::error("Frame validation failed");
            if (validation.error_code == ErrorCode::SENSOR_ASYMMETRY) {
                Logger::error("Sensor count asymmetry detected");
            } else if (validation.error_code == ErrorCode::UNREALISTIC_JUMP) {
                Logger::error("Unrealistic sensor value jump detected");
            }
            return RadiationResult::invalid();
        }
        
        // Execute physics engine
        RadiationResult result = engine.compute(frame, inr_ref, inr_flux);
        
        if (result.valid) {
            frame_count++;
            Logger::debug("Frame processed successfully");
        }
        
        return result;
    }
    
    /**
     * @brief Get current system status
     * 
     * @return SystemStatus structure with diagnostic information
     */
    SystemStatus getStatus() const {
        SystemStatus status;
        status.configured = configured;
        status.frames_processed = frame_count;
        status.sensor_ref_count = hub.getSensorCount(SensorType::REFERENCE);
        status.sensor_flux_count = hub.getSensorCount(SensorType::FLUX);
        return status;
    }
    
    /**
     * @brief Reset the system and clear all buffered data
     */
    void reset() {
        hub.reset();
        inr_ref.reset();
        inr_flux.reset();
        frame_count = 0;
        Logger::info("System reset complete");
    }
};

} // namespace FiaPhy

#endif // FIAPHY_H
