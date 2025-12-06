/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

/**
 * @file PhysicsEngine.h
 * @brief Core DTDSS computation engine (Dual pipeline architecture)
 * 
 * Implements the heart of the research paper:
 * - Reference Path: Kasten-Czeplak cloud proxy model (baseline)
 * - Reactive Path: Differential temperature + INR + Newton's Law inversion
 * 
 * This is where sensor data transforms into solar radiation estimates.
 */

#ifndef FIAPHY_PHYSICSENGINE_H
#define FIAPHY_PHYSICSENGINE_H

#include "Types.h"
#include "Constants.h"
#include "../physics/Thermodynamics.h"
#include "../physics/SolarGeometry.h"
#include "../filters/INR.h"
#include "../platform/Logger.h"
#include <math.h>

namespace FiaPhy {

/**
 * @class PhysicsEngine
 * @brief Executes the DTDSS algorithm to reconstruct solar radiation
 * 
 * Pipeline Architecture:
 * 
 * REFERENCE PATH (Slow, Stable):
 * [Ventilated Sensor] → [Humidity → Cloud Proxy] → [Kasten-Czeplak] → Baseline GHI
 * 
 * REACTIVE PATH (Fast, Responsive):
 * [Black-Body Sensor] → [ΔT] → [INR Filter] → [dT/dt] → [Newton's Law Inverse] → Instantaneous GHI
 * 
 * FUSION:
 * Result = weighted average or sanity-bounded selection
 */
class PhysicsEngine {
private:
    float latitude;
    float longitude;
    float altitude_m;
    
    CalibrationParams calibration;
    
    /**
     * @brief Reference Path: Estimate baseline radiation from humidity
     * 
     * Paper Section 3.1 - Kasten-Czeplak model:
     * G = G_cs * (1 - A * N^B)
     * where A = 0.75, B = 3.4
     * 
     * Cloud proxy from humidity:
     * N ≈ (RH / 100)^k
     * where k is typically 1.5-2.0 (we use exponent directly applied)
     * 
     * @param ref_frame Reference sensor reading
     * @param clear_sky_ghi Clear-sky theoretical maximum
     * @return Baseline GHI estimate and cloud fraction
     */
    struct BaselineResult {
        float ghi_Wm2;
        float cloud_fraction;
    };
    
    BaselineResult computeReferencePath(const SensorFrame& ref_frame, float clear_sky_ghi) {
        BaselineResult result;
        
        // Cloud proxy from humidity (Paper Section 3.1)
        float humidity_fraction = ref_frame.humidity_RH / 100.0f;
        
        // Apply cloud proxy exponent: N = (RH/100)^k
        // Note: CLOUD_PROXY_EXPONENT already includes the k factor
        float cloud_proxy_raw = powf(humidity_fraction, Constants::CLOUD_PROXY_EXPONENT);
        
        // Clamp to [0, 1]
        result.cloud_fraction = cloud_proxy_raw > 1.0f ? 1.0f : 
                                (cloud_proxy_raw < 0.0f ? 0.0f : cloud_proxy_raw);
        
        // Kasten-Czeplak attenuation: G = G_cs * (1 - 0.75 * N^3.4)
        // But we've already applied k to get N, so we use B directly
        float attenuation = 1.0f - Constants::KASTEN_A * 
                           powf(result.cloud_fraction, Constants::KASTEN_B);
        
        // Ensure non-negative
        if (attenuation < 0.0f) attenuation = 0.0f;
        
        result.ghi_Wm2 = clear_sky_ghi * attenuation;
        
        return result;
    }
    
    /**
     * @brief Reactive Path: Compute radiation from differential temperature
     * 
     * Paper Section 4.2 - Newton's Law Inverse:
     * G = (h_c / α) * [(T_flux - T_ref) + τ * dT/dt - T_rise]
     * 
     * Where:
     * - h_c = convective heat transfer coefficient (altitude-adaptive)
     * - α = solar absorptivity (typically 0.85-0.95 for black coating)
     * - τ = thermal time constant (sensor inertia)
     * - dT/dt = temperature derivative from INR filter
     * - T_rise = self-heating offset (electronic power dissipation)
     * 
     * This inverts Newton's Law of Cooling to solve for absorbed flux.
     * 
     * @param ref_frame Reference sensor (ambient temperature)
     * @param flux_frame Flux sensor (heated by radiation)
     * @param inr_filter INR filter instance (provides derivative)
     * @param air_density Air density in kg/m³ (for h_c calculation)
     * @return Instantaneous GHI estimate in W/m²
     */
    float computeReactivePath(const SensorFrame& ref_frame, 
                             const SensorFrame& flux_frame,
                             INRFilter& inr_filter,
                             float air_density) {
        // Temperature differential (Paper Section 3.2)
        float delta_T = flux_frame.temperature_C - ref_frame.temperature_C;
        
        // INR filtered derivative (dT/dt in °C/s)
        float temp_derivative = inr_filter.getDerivative();
        
        // Convective heat transfer coefficient (Paper Section 2.5)
        // h_c scales with sqrt(ρ) for altitude compensation
        float h_c = Thermodynamics::convectiveHeatTransferCoefficient(air_density);
        
        // Solar Air Excess (Paper Section 4.2)
        // T_sol = ΔT + τ*(dT/dt) - T_rise
        float sol_air_excess = delta_T + 
                               calibration.thermal_time_constant_s * temp_derivative - 
                               calibration.self_heating_offset_C;
        
        // Solve for radiation: G = (h_c / α) * T_sol
        float ghi = (h_c / calibration.solar_absorptivity) * sol_air_excess;
        
        // Physical constraint: radiation cannot be negative
        if (ghi < 0.0f) ghi = 0.0f;
        
        return ghi;
    }
    
public:
    /**
     * @brief Construct physics engine
     */
    PhysicsEngine() : latitude(0), longitude(0), altitude_m(0) {}
    
    /**
     * @brief Set geographic location for solar geometry calculations
     * 
     * @param lat Latitude in decimal degrees
     * @param lon Longitude in decimal degrees
     */
    void setLocation(float lat, float lon) {
        latitude = lat;
        longitude = lon;
    }
    
    /**
     * @brief Set calibration parameters
     * 
     * @param cal Calibration structure
     */
    void setCalibration(const CalibrationParams& cal) {
        calibration = cal;
    }
    
    /**
     * @brief Main computation: Execute both pipelines and fuse results
     * 
     * @param frame Sensor frame (contains both ref and flux data)
     * @param inr_ref INR filter for reference sensor
     * @param inr_flux INR filter for flux sensor
     * @return RadiationResult with GHI, heat flux, and diagnostics
     */
    SolarFlux compute(const SensorFrame& ref_frame, 
                      INRFilter& inr_ref, 
                      INRFilter& inr_flux) {
        SolarFlux result;
        
        // Step 1: Calculate atmospheric state (altitude-adaptive)
        AtmosphericState atmos = Thermodynamics::calculateAtmosphericState(
            ref_frame.temperature_C,
            ref_frame.humidity_RH,
            ref_frame.pressure_hPa
        );
        
        // Step 2: Apply INR filtering to flux sensor temperature
        float filtered_flux_temp = inr_flux.process(ref_frame.temperature_C);  // Using ref as proxy for now
        
        // Step 3: Calculate clear-sky radiation (solar geometry)
        // TODO: Implement full solar position algorithm
        // For now, use simplified model based on time of day
        float clear_sky_ghi = 1000.0f;  // Placeholder: ~1000 W/m² typical max
        
        // Step 4: REFERENCE PATH (baseline from humidity/cloud proxy)
        BaselineResult baseline = computeReferencePath(ref_frame, clear_sky_ghi);
        
        result.irradiance_Wm2 = baseline.ghi_Wm2;
        result.cloud_fraction = baseline.cloud_fraction;
        
        // Step 5: REACTIVE PATH (differential temperature analysis)
        // Note: Currently disabled because we need proper flux sensor data structure
        // float reactive_ghi = computeReactivePath(ref_frame, flux_frame, inr_flux, atmos.air_density_kg_m3);
        
        // Step 6: FUSION (for now, just use reference path)
        // In production, this would be:
        // if (abs(reactive_ghi - baseline.ghi_Wm2) < threshold) {
        //     result.irradiance_Wm2 = weighted_average(reactive_ghi, baseline.ghi_Wm2);
        //     result.confidence = 0.9f;
        // } else {
        //     // Large disagreement → trust baseline, flag uncertainty
        //     result.confidence = 0.5f;
        // }
        
        result.confidence = 0.7f;  // Placeholder
        
        return result;
    }
};

} // namespace FiaPhy

#endif // FIAPHY_PHYSICSENGINE_H
