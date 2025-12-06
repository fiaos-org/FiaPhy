/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

//==============================================================================
// FiaPhy Core Types - Fundamental Data Structures
// Defines all thermodynamic state variables and sensor data formats
//==============================================================================

#ifndef FIAPHY_TYPES_H
#define FIAPHY_TYPES_H

#include <stdint.h>

namespace FiaPhy {

// Sensor reading structure - raw input from hardware
struct SensorReading {
    float temperature_C;    // Temperature in Celsius
    float humidity_RH;      // Relative humidity (0-100%)
    float pressure_hPa;     // Atmospheric pressure in hectopascals
    uint8_t sensor_id;      // Sensor identifier (0-255)
    uint32_t timestamp_ms;  // Millisecond timestamp (optional, 0 if unused)
    
    bool isValid() const {
        return temperature_C > -100.0f && temperature_C < 100.0f &&
               humidity_RH >= 0.0f && humidity_RH <= 100.0f &&
               pressure_hPa > 300.0f && pressure_hPa < 1200.0f;
    }
};

// Complete sensor frame - synchronized triplet of T/H/P
struct SensorFrame {
    float temperature_C;
    float humidity_RH;
    float pressure_hPa;
    uint8_t sensor_id;
    bool complete;          // True when all three values synchronized
    
    SensorFrame() : temperature_C(0), humidity_RH(0), pressure_hPa(0), 
                    sensor_id(0), complete(false) {}
    
    void reset() {
        temperature_C = 0;
        humidity_RH = 0;
        pressure_hPa = 0;
        complete = false;
    }
};

// Atmospheric state - derived thermodynamic properties
struct AtmosphericState {
    float air_density_kg_m3;        // ρ_moist
    float vapor_pressure_hPa;       // e (partial pressure of water vapor)
    float saturation_pressure_hPa;  // e_s
    float specific_enthalpy_kJ_kg;  // h (total energy content)
    float mixing_ratio;             // x (kg_vapor / kg_dry_air)
    float temperature_K;            // Absolute temperature
};

// Solar flux computation result
struct SolarFlux {
    // Radiation components
    float irradiance_Wm2;          // Global Horizontal Irradiance (GHI)
    float heat_flux_Wm2;           // Convective heat flux
    float sol_air_excess_C;        // T_sol (excess temperature)
    float cloud_fraction;          // Estimated cloud cover (0-1)
    float confidence;              // Result confidence (0-1)
    
    // Solar geometry parameters
    int day_of_year;               // Day number (1-365)
    float hour_angle_deg;          // Hour angle in degrees
    float zenith_angle_deg;        // Solar zenith angle in degrees
    float elevation_angle_deg;     // Solar elevation angle in degrees
    float azimuth_angle_deg;       // Solar azimuth angle in degrees
    bool sun_is_up;                // True if sun elevation > 0
    float clear_sky_ghi_Wm2;       // Clear-sky GHI estimate
    
    SolarFlux() : irradiance_Wm2(0), heat_flux_Wm2(0), 
                  sol_air_excess_C(0), cloud_fraction(0), confidence(0),
                  day_of_year(0), hour_angle_deg(0), zenith_angle_deg(0),
                  elevation_angle_deg(0), azimuth_angle_deg(0), sun_is_up(false),
                  clear_sky_ghi_Wm2(0) {}
};

// INR filter state (Inertial Noise Reduction)
struct INRState {
    float filtered_value;
    float previous_filtered;
    float derivative;
    float projected_value;
    float alpha;                    // Adaptive smoothing factor
    uint8_t buffer_index;
    static constexpr uint8_t BUFFER_SIZE = 10;
    float circular_buffer[BUFFER_SIZE];
    
    INRState() : filtered_value(0), previous_filtered(0), derivative(0),
                 projected_value(0), alpha(0.2f), buffer_index(0) {
        for(uint8_t i = 0; i < BUFFER_SIZE; i++) circular_buffer[i] = 0;
    }
};

// Calibration parameters (user-configurable or auto-detected)
struct CalibrationParams {
    float thermal_time_constant_s;  // τ (sensor thermal inertia)
    float solar_absorptivity;       // α (black body coating)
    float convective_area_m2;       // A_s (sensor surface area)
    float heat_capacity_J_K;        // m·Cp (thermal mass)
    float self_heating_offset_C;    // T_rise (electrical self-heating)
    
    // Default values for BME280 in typical enclosure
    CalibrationParams() : 
        thermal_time_constant_s(30.0f),
        solar_absorptivity(0.90f),
        convective_area_m2(6.25e-6f),  // 2.5mm × 2.5mm
        heat_capacity_J_K(0.5f),
        self_heating_offset_C(0.8f) {}
};

// Error codes for validation failures
enum class ErrorCode : uint8_t {
    OK = 0,
    SENSOR_ASYMMETRY,           // Unequal sensor counts (e.g., 2T + 3H + 2P)
    INCOMPLETE_TRIPLET,         // Missing T, H, or P value
    VALUE_OUT_OF_RANGE,         // Physically impossible reading
    UNREALISTIC_JUMP,           // Value changed too rapidly (>20°C/s)
    INSUFFICIENT_SENSORS,       // Less than 2 sensor pairs
    BUFFER_OVERFLOW,            // Too many sensors (>8 pairs)
    CALIBRATION_FAILED,         // Auto-calibration could not converge
    INVALID_STATE               // Internal consistency check failed
};

// Validation result structure
struct ValidationResult {
    bool valid;                 // Overall validation status
    ErrorCode error_code;       // Specific error if !valid
    const char* message;
    uint8_t faulty_sensor_id;
    
    ValidationResult() : valid(true), error_code(ErrorCode::OK), message(""), faulty_sensor_id(0) {}
    
    bool isOk() const { return valid && error_code == ErrorCode::OK; }
};

// Sensor type enumeration
enum class SensorType : uint8_t {
    REFERENCE = 0,  // Ventilated, shielded sensor
    FLUX = 1        // Black-body, absorptive sensor
};

// Extended sensor frame with completion tracking (v1.0.1+)
struct SensorFrameEx {
    float temperature_C;
    float humidity_RH;
    float pressure_hPa;
    uint8_t sensor_id;
    
    // Completion flags
    bool has_temperature;
    bool has_humidity;
    bool has_pressure;
    bool complete;
    
    SensorFrame() : temperature_C(0), humidity_RH(0), pressure_hPa(0), 
                    sensor_id(0), has_temperature(false), has_humidity(false),
                    has_pressure(false), complete(false) {}
    
    void reset() {
        temperature_C = 0;
        humidity_RH = 0;
        pressure_hPa = 0;
        has_temperature = false;
        has_humidity = false;
        has_pressure = false;
        complete = false;
    }
    
    void checkCompletion() {
        complete = has_temperature && has_humidity && has_pressure;
    }
};

// Radiation computation result
struct RadiationResult {
    bool valid;
    float ghi_Wm2;              // Global Horizontal Irradiance
    float heat_flux_Wm2;        // Convective heat flux
    float temp_differential_C;  // T_flux - T_ref
    float temp_derivative_C_s;  // dT/dt
    float sol_air_excess_C;     // Solar air temperature excess
    float baseline_ghi_Wm2;     // Kasten-Czeplak baseline
    float cloud_proxy;          // Cloud fraction estimate
    float air_density_kg_m3;    // Calculated air density
    float vapor_pressure_hPa;   // Vapor pressure
    uint32_t timestamp_ms;
    
    RadiationResult() : valid(false), ghi_Wm2(0), heat_flux_Wm2(0),
                        temp_differential_C(0), temp_derivative_C_s(0), sol_air_excess_C(0),
                        baseline_ghi_Wm2(0), cloud_proxy(0), air_density_kg_m3(0),
                        vapor_pressure_hPa(0), timestamp_ms(0) {}
    
    static RadiationResult invalid() {
        RadiationResult r;
        r.valid = false;
        return r;
    }
};

// System status structure
struct SystemStatus {
    bool configured;
    uint32_t frames_processed;
    uint8_t sensor_ref_count;
    uint8_t sensor_flux_count;
    
    SystemStatus() : configured(false), frames_processed(0),
                     sensor_ref_count(0), sensor_flux_count(0) {}
};

// Configuration structure
struct Configuration {
    float latitude;
    float longitude;
    float altitude_m;
    
    Configuration() : latitude(0), longitude(0), altitude_m(0) {}
};

} // namespace FiaPhy

#endif // FIAPHY_TYPES_H
