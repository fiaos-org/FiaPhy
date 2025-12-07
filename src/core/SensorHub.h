/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

/**
 * @file SensorHub.h
 * @brief Asynchronous sensor data synchronization and validation
 * 
 * Synchronizes sensor readings arriving at unpredictable times into complete triplets (T+H+P).
 * Waits indefinitely for complete frames. Validates unrealistic value jumps.
 * Supports 1-8 sensor pairs.
 */

#ifndef FIAPHY_SENSORHUB_H
#define FIAPHY_SENSORHUB_H

#include "Types.h"
#include "Constants.h"
#include "../platform/Logger.h"
#include <stdint.h>

namespace FiaPhy {

/**
 * @class SensorHub
 * @brief Manages asynchronous sensor data ingestion and frame synchronization
 */
class SensorHub {
private:
    // Partial reading buffers
    SensorFrame ref_buffer[Constants::MAX_SENSOR_PAIRS];
    SensorFrame flux_buffer[Constants::MAX_SENSOR_PAIRS];
    
    // Active sensor tracking (bitfield)
    uint8_t ref_sensor_mask;
    uint8_t flux_sensor_mask;
    
    // Parameter count tracking
    uint8_t ref_temp_count;
    uint8_t ref_humidity_count;
    uint8_t ref_pressure_count;
    uint8_t flux_temp_count;
    uint8_t flux_humidity_count;
    uint8_t flux_pressure_count;
    
    // Previous values for jump detection
    float prev_ref_temp[Constants::MAX_SENSOR_PAIRS];
    float prev_ref_humidity[Constants::MAX_SENSOR_PAIRS];
    float prev_ref_pressure[Constants::MAX_SENSOR_PAIRS];
    float prev_flux_temp[Constants::MAX_SENSOR_PAIRS];
    float prev_flux_humidity[Constants::MAX_SENSOR_PAIRS];
    float prev_flux_pressure[Constants::MAX_SENSOR_PAIRS];
    
    bool prev_values_initialized[Constants::MAX_SENSOR_PAIRS * 2];
    
    // Frame state
    bool frame_ready;
    uint32_t frame_counter;
    
    /**
     * @brief Check if value change exceeds maximum allowed
     */
    bool isUnrealisticJump(float new_val, float prev_val, float max_jump, 
                           uint8_t sensor_id, bool initialized) {
        if (!initialized) {
            return false;
        }
        
        float delta = new_val > prev_val ? (new_val - prev_val) : (prev_val - new_val);
        
        if (delta > max_jump) {
            Logger::warn("Unrealistic jump detected on sensor");
            return true;
        }
        
        return false;
    }
    
    /**
     * @brief Validate physical range
     */
    bool isValidRange(float temp_C, float humidity, float pressure_hPa) {
        return temp_C >= Constants::MIN_VALID_TEMP_C && 
               temp_C <= Constants::MAX_VALID_TEMP_C &&
               humidity >= Constants::MIN_VALID_HUMIDITY && 
               humidity <= Constants::MAX_VALID_HUMIDITY &&
               pressure_hPa >= Constants::MIN_VALID_PRESSURE_HPA && 
               pressure_hPa <= Constants::MAX_VALID_PRESSURE_HPA;
    }
    
    /**
     * @brief Check if all sensors have complete triplets
     */
    void checkFrameCompletion() {
        uint8_t ref_complete = 0;
        uint8_t flux_complete = 0;
        
        for (uint8_t i = 0; i < Constants::MAX_SENSOR_PAIRS; i++) {
            if ((ref_sensor_mask & (1 << i)) && ref_buffer[i].complete) {
                ref_complete++;
            }
            if ((flux_sensor_mask & (1 << i)) && flux_buffer[i].complete) {
                flux_complete++;
            }
        }
        
        uint8_t ref_active = countBits(ref_sensor_mask);
        uint8_t flux_active = countBits(flux_sensor_mask);
        
        frame_ready = (ref_complete == ref_active) && 
                      (flux_complete == flux_active) &&
                      (ref_active > 0) && (flux_active > 0);
    }
    
    /**
     * @brief Count set bits in byte
     */
    uint8_t countBits(uint8_t value) const {
        uint8_t count = 0;
        while (value) {
            count += value & 1;
            value >>= 1;
        }
        return count;
    }
    
public:
    SensorHub() : 
        ref_sensor_mask(0), flux_sensor_mask(0),
        ref_temp_count(0), ref_humidity_count(0), ref_pressure_count(0),
        flux_temp_count(0), flux_humidity_count(0), flux_pressure_count(0),
        frame_ready(false), frame_counter(0) {
        
        for (uint8_t i = 0; i < Constants::MAX_SENSOR_PAIRS; i++) {
            ref_buffer[i].reset();
            flux_buffer[i].reset();
            prev_values_initialized[i] = false;
            prev_values_initialized[i + Constants::MAX_SENSOR_PAIRS] = false;
        }
    }
    
    /**
     * @brief Feed temperature reading
     */
    void feedTemperature(SensorType type, uint8_t sensor_id, float temp_C) {
        if (sensor_id >= Constants::MAX_SENSOR_PAIRS) {
            Logger::error("Sensor ID exceeds maximum");
            return;
        }
        
        SensorFrame* buffer = (type == SensorType::REFERENCE) ? ref_buffer : flux_buffer;
        uint8_t* sensor_mask = (type == SensorType::REFERENCE) ? &ref_sensor_mask : &flux_sensor_mask;
        uint8_t* temp_count = (type == SensorType::REFERENCE) ? &ref_temp_count : &flux_temp_count;
        float* prev_temp = (type == SensorType::REFERENCE) ? prev_ref_temp : prev_flux_temp;
        uint8_t prev_idx = (type == SensorType::REFERENCE) ? sensor_id : (sensor_id + Constants::MAX_SENSOR_PAIRS);
        
        if (isUnrealisticJump(temp_C, prev_temp[sensor_id], Constants::MAX_TEMP_JUMP_C_PER_S, 
                              sensor_id, prev_values_initialized[prev_idx])) {
            Logger::error("Temperature jump validation failed");
            return;
        }
        
        buffer[sensor_id].temperature_C = temp_C;
        buffer[sensor_id].has_temperature = true;
        buffer[sensor_id].sensor_id = sensor_id;
        prev_temp[sensor_id] = temp_C;
        prev_values_initialized[prev_idx] = true;
        
        *sensor_mask |= (1 << sensor_id);
        
        if (!buffer[sensor_id].has_humidity && !buffer[sensor_id].has_pressure) {
            (*temp_count)++;
        }
        
        buffer[sensor_id].checkCompletion();
        checkFrameCompletion();
    }
    
    /**
     * @brief Feed humidity reading
     */
    void feedHumidity(SensorType type, uint8_t sensor_id, float humidity_percent) {
        if (sensor_id >= Constants::MAX_SENSOR_PAIRS) {
            Logger::error("Sensor ID exceeds maximum");
            return;
        }
        
        SensorFrame* buffer = (type == SensorType::REFERENCE) ? ref_buffer : flux_buffer;
        uint8_t* sensor_mask = (type == SensorType::REFERENCE) ? &ref_sensor_mask : &flux_sensor_mask;
        uint8_t* humidity_count = (type == SensorType::REFERENCE) ? &ref_humidity_count : &flux_humidity_count;
        float* prev_humidity = (type == SensorType::REFERENCE) ? prev_ref_humidity : prev_flux_humidity;
        uint8_t prev_idx = (type == SensorType::REFERENCE) ? sensor_id : (sensor_id + Constants::MAX_SENSOR_PAIRS);
        
        if (isUnrealisticJump(humidity_percent, prev_humidity[sensor_id], 
                              Constants::MAX_HUMIDITY_JUMP_PERCENT_PER_S, 
                              sensor_id, prev_values_initialized[prev_idx])) {
            Logger::error("Humidity jump validation failed");
            return;
        }
        
        buffer[sensor_id].humidity_RH = humidity_percent;
        buffer[sensor_id].has_humidity = true;
        buffer[sensor_id].sensor_id = sensor_id;
        prev_humidity[sensor_id] = humidity_percent;
        prev_values_initialized[prev_idx] = true;
        
        *sensor_mask |= (1 << sensor_id);
        
        if (!buffer[sensor_id].has_temperature && !buffer[sensor_id].has_pressure) {
            (*humidity_count)++;
        }
        
        buffer[sensor_id].checkCompletion();
        checkFrameCompletion();
    }
    
    /**
     * @brief Feed pressure reading
     */
    void feedPressure(SensorType type, uint8_t sensor_id, float pressure_hPa) {
        if (sensor_id >= Constants::MAX_SENSOR_PAIRS) {
            Logger::error("Sensor ID exceeds maximum");
            return;
        }
        
        SensorFrame* buffer = (type == SensorType::REFERENCE) ? ref_buffer : flux_buffer;
        uint8_t* sensor_mask = (type == SensorType::REFERENCE) ? &ref_sensor_mask : &flux_sensor_mask;
        uint8_t* pressure_count = (type == SensorType::REFERENCE) ? &ref_pressure_count : &flux_pressure_count;
        float* prev_pressure = (type == SensorType::REFERENCE) ? prev_ref_pressure : prev_flux_pressure;
        uint8_t prev_idx = (type == SensorType::REFERENCE) ? sensor_id : (sensor_id + Constants::MAX_SENSOR_PAIRS);
        
        if (isUnrealisticJump(pressure_hPa, prev_pressure[sensor_id], 
                              Constants::MAX_PRESSURE_JUMP_HPA_PER_S, 
                              sensor_id, prev_values_initialized[prev_idx])) {
            Logger::error("Pressure jump validation failed");
            return;
        }
        
        buffer[sensor_id].pressure_hPa = pressure_hPa;
        buffer[sensor_id].has_pressure = true;
        buffer[sensor_id].sensor_id = sensor_id;
        prev_pressure[sensor_id] = pressure_hPa;
        prev_values_initialized[prev_idx] = true;
        
        *sensor_mask |= (1 << sensor_id);
        
        if (!buffer[sensor_id].has_temperature && !buffer[sensor_id].has_humidity) {
            (*pressure_count)++;
        }
        
        buffer[sensor_id].checkCompletion();
        checkFrameCompletion();
    }
    
    /**
     * @brief Check if complete synchronized frame is ready
     */
    bool isFrameReady() const {
        return frame_ready;
    }
    
    /**
     * @brief Retrieve synchronized sensor pair (Reference + Flux)
     * @return DifferentialFrame containing both sensors for DTDSS algorithm
     */
    DifferentialFrame getDifferentialFrame() {
        DifferentialFrame result;
        
        if (!frame_ready) {
            Logger::warn("getDifferentialFrame() called before frame ready");
            return result; 
        }
        
        // Find the first valid pair (in a multi-sensor setup, typically index 0)
        for (uint8_t i = 0; i < Constants::MAX_SENSOR_PAIRS; i++) {
            if ((ref_sensor_mask & (1 << i)) && ref_buffer[i].complete &&
                (flux_sensor_mask & (1 << i)) && flux_buffer[i].complete) {
                
                result.ref = ref_buffer[i];
                result.flux = flux_buffer[i];
                result.valid = true;
                break;
            }
        }
        
        // Clear flags for next cycle
        frame_counter++;
        reset();
        
        return result;
    }
    
    /**
     * @brief Retrieve complete sensor frame (call only after isFrameReady returns true)
     * @deprecated Use getDifferentialFrame() for full DTDSS algorithm
     */
    SensorFrame getFrame() {
        if (!frame_ready) {
            Logger::warn("getFrame() called before frame ready");
        }
        
        SensorFrame result;
        
        for (uint8_t i = 0; i < Constants::MAX_SENSOR_PAIRS; i++) {
            if ((ref_sensor_mask & (1 << i)) && ref_buffer[i].complete) {
                result = ref_buffer[i];
                break;
            }
        }
        
        frame_counter++;
        reset();
        
        return result;
    }
    
    /**
     * @brief Validate frame for asymmetry and range
     */
    ValidationResult validateFrame(const SensorFrame& frame) {
        ValidationResult result;
        
        if (ref_temp_count != ref_humidity_count || 
            ref_humidity_count != ref_pressure_count) {
            result.valid = false;
            result.error_code = ErrorCode::SENSOR_ASYMMETRY;
            result.message = "Reference sensor counts asymmetric (T != H != P)";
            Logger::error(result.message);
            return result;
        }
        
        if (flux_temp_count != flux_humidity_count || 
            flux_humidity_count != flux_pressure_count) {
            result.valid = false;
            result.error_code = ErrorCode::SENSOR_ASYMMETRY;
            result.message = "Flux sensor counts asymmetric (T != H != P)";
            Logger::error(result.message);
            return result;
        }
        
        uint8_t ref_count = countBits(ref_sensor_mask);
        uint8_t flux_count = countBits(flux_sensor_mask);
        
        if (ref_count < Constants::MIN_SENSOR_PAIRS || flux_count < Constants::MIN_SENSOR_PAIRS) {
            result.valid = false;
            result.error_code = ErrorCode::INSUFFICIENT_SENSORS;
            result.message = "Minimum sensor pair count not met";
            Logger::error(result.message);
            return result;
        }
        
        if (!isValidRange(frame.temperature_C, frame.humidity_RH, frame.pressure_hPa)) {
            result.valid = false;
            result.error_code = ErrorCode::VALUE_OUT_OF_RANGE;
            result.message = "Sensor value outside valid range";
            Logger::error(result.message);
            return result;
        }
        
        result.valid = true;
        result.error_code = ErrorCode::OK;
        result.message = "Validation passed";
        return result;
    }
    
    /**
     * @brief Get active sensor count
     */
    uint8_t getSensorCount(SensorType type) const {
        return countBits(type == SensorType::REFERENCE ? ref_sensor_mask : flux_sensor_mask);
    }
    
    /**
     * @brief Reset all buffers and state
     */
    void reset() {
        ref_sensor_mask = 0;
        flux_sensor_mask = 0;
        ref_temp_count = 0;
        ref_humidity_count = 0;
        ref_pressure_count = 0;
        flux_temp_count = 0;
        flux_humidity_count = 0;
        flux_pressure_count = 0;
        frame_ready = false;
        
        for (uint8_t i = 0; i < Constants::MAX_SENSOR_PAIRS; i++) {
            ref_buffer[i].reset();
            flux_buffer[i].reset();
        }
    }
};

} // namespace FiaPhy

#endif // FIAPHY_SENSORHUB_H
