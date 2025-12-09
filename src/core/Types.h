/*Copyright(c) 2025 FIA Operating Systems, All Rights Reserved. Cloud data: https://www.fiaos.org/data*/

#ifndef FIAPHY_TYPES_H
#define FIAPHY_TYPES_H

#include <stdint.h>

namespace FiaPhy {

struct SensorReading {
    float temperature_C;
    float humidity_RH;
    float pressure_hPa;
    uint8_t sensor_id;
    uint32_t timestamp_ms;
    
    bool isValid() const {
        return temperature_C > -100.0f && temperature_C < 100.0f &&
               humidity_RH >= 0.0f && humidity_RH <= 100.0f &&
               pressure_hPa > 300.0f && pressure_hPa < 1200.0f;
    }
};


struct SensorFrame {
    float temperature_C;
    float humidity_RH;
    float pressure_hPa;
    uint8_t sensor_id;
    bool complete;

    bool has_temperature;
    bool has_humidity;
    bool has_pressure;
    
    SensorFrame() : temperature_C(0), humidity_RH(0), pressure_hPa(0), 
                    sensor_id(0), complete(false), has_temperature(false),
                    has_humidity(false), has_pressure(false) {}
    
    void reset() {
        temperature_C = 0;
        humidity_RH = 0;
        pressure_hPa = 0;
        complete = false;
        has_temperature = false;
        has_humidity = false;
        has_pressure = false;
    }
    
    void checkCompletion() {
        complete = has_temperature && has_humidity && has_pressure;
    }
};


struct DifferentialFrame {
    SensorFrame ref;
    SensorFrame flux;
    bool valid;
    
    DifferentialFrame() : valid(false) {}
};


struct AtmosphericState {
    float air_density_kg_m3;
    float vapor_pressure_hPa;
    float saturation_pressure_hPa;
    float specific_enthalpy_kJ_kg;
    float mixing_ratio;
    float temperature_K;
};


struct SolarFlux {
    float irradiance_Wm2;
    float heat_flux_Wm2;
    float sol_air_excess_C;
    float cloud_fraction;
    float confidence;

    int day_of_year;
    float hour_angle_deg;
    float zenith_angle_deg;
    float elevation_angle_deg;
    float azimuth_angle_deg;
    bool sun_is_up;
    float clear_sky_ghi_Wm2;
    
    SolarFlux() : irradiance_Wm2(0), heat_flux_Wm2(0), 
                  sol_air_excess_C(0), cloud_fraction(0), confidence(0),
                  day_of_year(0), hour_angle_deg(0), zenith_angle_deg(0),
                  elevation_angle_deg(0), azimuth_angle_deg(0), sun_is_up(false),
                  clear_sky_ghi_Wm2(0) {}
};


struct INRState {
    float filtered_value;
    float previous_filtered;
    float derivative;
    float projected_value;
    float alpha;
    uint8_t buffer_index;
    static constexpr uint8_t BUFFER_SIZE = 10;
    float circular_buffer[BUFFER_SIZE];
    
    INRState() : filtered_value(0), previous_filtered(0), derivative(0),
                 projected_value(0), alpha(0.2f), buffer_index(0) {
        for(uint8_t i = 0; i < BUFFER_SIZE; i++) circular_buffer[i] = 0;
    }
};


struct CalibrationParams {
    float thermal_time_constant_s;
    float solar_absorptivity;
    float convective_area_m2;
    float heat_capacity_J_K;
    float self_heating_offset_C;

    CalibrationParams() :
        thermal_time_constant_s(30.0f),
        solar_absorptivity(0.90f),
        convective_area_m2(6.25e-6f),
        heat_capacity_J_K(0.5f),
        self_heating_offset_C(0.8f) {}
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
    ErrorCode error_code;
    const char* message;
    uint8_t faulty_sensor_id;
    
    ValidationResult() : valid(true), error_code(ErrorCode::OK), message(""), faulty_sensor_id(0) {}
    
    bool isOk() const { return valid && error_code == ErrorCode::OK; }
};


enum class SensorType : uint8_t {
    REFERENCE = 0,
    FLUX = 1
};


struct RadiationResult {
    bool valid;
    float ghi_Wm2;
    float heat_flux_Wm2;
    float temp_differential_C;
    float temp_derivative_C_s;
    float sol_air_excess_C;
    float baseline_ghi_Wm2;
    float cloud_proxy;
    float air_density_kg_m3;
    float vapor_pressure_hPa;
    float confidence;
    uint32_t timestamp_ms;
    
    RadiationResult() : valid(false), ghi_Wm2(0), heat_flux_Wm2(0),
                        temp_differential_C(0), temp_derivative_C_s(0), sol_air_excess_C(0),
                        baseline_ghi_Wm2(0), cloud_proxy(0), air_density_kg_m3(0),
                        vapor_pressure_hPa(0), confidence(0.0f), timestamp_ms(0) {}
    
    static RadiationResult invalid() {
        RadiationResult r;
        r.valid = false;
        return r;
    }
};


struct SystemStatus {
    bool configured;
    uint32_t frames_processed;
    uint8_t sensor_ref_count;
    uint8_t sensor_flux_count;
    
    SystemStatus() : configured(false), frames_processed(0),
                     sensor_ref_count(0), sensor_flux_count(0) {}
};


struct Configuration {
    float latitude;
    float longitude;
    float altitude_m;
    
    Configuration() : latitude(0), longitude(0), altitude_m(0) {}
};


}

#endif
