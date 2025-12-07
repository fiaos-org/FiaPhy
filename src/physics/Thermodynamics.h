//==============================================================================
// Thermodynamics Module - Moist Air Physics
// 
// Implements Section 2 of research paper:
// - Air density calculation (altitude-adaptive)
// - Vapor pressure derivation (Magnus formula)
// - Specific enthalpy computation
// - All calculations use measured P/T/RH (no standard atmosphere assumptions)
//
// IMPLEMENTATION NOTE:
// This reference implementation uses floating-point arithmetic for clarity,
// precision, and compatibility with modern 32-bit microcontrollers (ESP32, 
// ARM Cortex-M4). While the algorithm is mathematically suitable for 
// fixed-point optimization on 8-bit architectures (replacing float operations
// with integer multiply-shift operations), the performance gains are negligible
// on contemporary IoT platforms with hardware FPUs. A fixed-point variant may
// be provided as a separate optimization layer if targeting legacy 8-bit MCUs.
//==============================================================================

#ifndef FIAPHY_THERMODYNAMICS_H
#define FIAPHY_THERMODYNAMICS_H

#include "../core/Types.h"
#include <math.h>

namespace FiaPhy {
namespace Thermodynamics {

// Calculate saturation vapor pressure using Magnus formula
// Research paper Section 2.3: e_s(T) = 6.112 * exp((17.67 * (T - 273.15)) / (T - 29.65))
// where T is in Kelvin
// Alduchov & Eskridge (1996) constants
// Input: Temperature in Celsius
// Output: Saturation vapor pressure in hPa
inline float saturationVaporPressure(float temp_C) {
    // Convert to Kelvin as per paper specification
    float temp_K = temp_C + 273.15f;
    
    // Paper formula: e_s(T) = 6.112 * exp((17.67 * (T - 273.15)) / (T - 29.65))
    float numerator = 17.67f * (temp_K - 273.15f);  // = 17.67 * temp_C
    float denominator = temp_K - 29.65f;            // NOT temp_C + 243.5
    
    return 6.112f * expf(numerator / denominator);
}

// Calculate actual vapor pressure from relative humidity
// Paper Section 2.3: e = e_s(T) * (RH / 100)
// Input: Temperature (°C), Relative Humidity (%)
// Output: Vapor pressure in hPa
inline float vaporPressure(float temp_C, float humidity_RH) {
    float e_sat = saturationVaporPressure(temp_C);
    return e_sat * (humidity_RH / 100.0f);
}

// Calculate moist air density (altitude-independent)
// This is the CORE innovation - uses measured pressure instead of altitude lookup
// 
// Paper Section 2.2: ρ_moist = (P/R_d*T) * (1 - (e/P)*(1-ε))
// where ε = R_d/R_v ≈ 0.622
// 
// Input: Pressure (hPa), Temperature (°C), Relative Humidity (%)
// Output: Air density in kg/m³
inline float airDensity(float pressure_hPa, float temp_C, float humidity_RH) {
    float temp_K = temp_C + 273.15f;
    float vapor_pressure_hPa = vaporPressure(temp_C, humidity_RH);
    
    // Convert pressure to Pascals (1 hPa = 100 Pa)
    float pressure_Pa = pressure_hPa * 100.0f;
    float vapor_pressure_Pa = vapor_pressure_hPa * 100.0f;
    
    // Dry air component: ρ_d = P_d / (R_d * T)
    float dry_density = (pressure_Pa - vapor_pressure_Pa) / 
                       (Constants::R_DRY_AIR * temp_K);
    
    // Water vapor component: ρ_v = e / (R_v * T)
    float vapor_density = vapor_pressure_Pa / 
                         (Constants::R_WATER_VAPOR * temp_K);
    
    return dry_density + vapor_density;
}

// Calculate humidity mixing ratio
// Paper Section 2.4: x = ε * e/(P - e) where ε = 0.622
// Units: kg_water_vapor / kg_dry_air
inline float mixingRatio(float pressure_hPa, float vapor_pressure_hPa) {
    return 0.622f * vapor_pressure_hPa / (pressure_hPa - vapor_pressure_hPa);
}

// Calculate specific enthalpy of moist air
// Paper Section 2.4: h = 1.006*t + x(2501 + 1.86*t) kJ/kg
// where:
//   h_a = 1.006 * t  (kJ/kg) - dry air enthalpy
//   h_v = 2501 + 1.86 * t  (kJ/kg) - water vapor enthalpy
//   2501 kJ/kg is latent heat of vaporization at 0°C
// 
// Input: Temperature (°C), mixing ratio (kg/kg)
// Output: Specific enthalpy in kJ/kg
inline float specificEnthalpy(float temp_C, float mixing_ratio) {
    float h_dry = 1.006f * temp_C;
    float h_vapor = 2501.0f + 1.86f * temp_C;
    return h_dry + mixing_ratio * h_vapor;
}

// Compute complete atmospheric state from raw sensor readings
// This is the primary interface for the physics engine
// Implements Section 2 of research paper
inline AtmosphericState calculateAtmosphericState(float temp_C, float humidity_RH, float pressure_hPa) {
    AtmosphericState state;
    
    state.temperature_K = temp_C + 273.15f;
    
    // Vapor pressure calculations (Section 2.3)
    state.saturation_pressure_hPa = saturationVaporPressure(temp_C);
    state.vapor_pressure_hPa = vaporPressure(temp_C, humidity_RH);
    
    // Air density (Section 2.2) - altitude-adaptive via measured pressure
    state.air_density_kg_m3 = airDensity(pressure_hPa, temp_C, humidity_RH);
    
    // Humidity mixing ratio (Section 2.4)
    state.mixing_ratio = mixingRatio(pressure_hPa, state.vapor_pressure_hPa);
    
    // Specific enthalpy (Section 2.4) - total energy content
    state.specific_enthalpy_kJ_kg = specificEnthalpy(temp_C, state.mixing_ratio);
    
    return state;
}

// Calculate convective heat transfer coefficient (h_c)
// Paper Section 2.5: h_c ∝ k_air * (ρV/μ)^0.5
// From Nusselt-Reynolds relationship: Nu ∝ Re^(1/2) * Pr^(1/3)
// Therefore: h_c ∝ ρ^0.5
// 
// Simplified model for BME280-sized sensor in natural convection:
// h_c ≈ 10 * sqrt(ρ / 1.225)  [W/(m²·K)]
// 
// Where 1.225 kg/m³ is sea-level reference density
// 
// Input: Air density in kg/m³
// Output: Convective heat transfer coefficient in W/(m²·K)
inline float convectiveHeatTransferCoefficient(float air_density_kg_m3) {
    // Base coefficient at sea level (empirical for BME280 package)
    constexpr float h_c_sea_level = 10.0f;  // W/(m²·K)
    constexpr float rho_sea_level = 1.225f; // kg/m³
    
    // Scale by square root of density ratio (from Nu ∝ Re^0.5)
    return h_c_sea_level * sqrtf(air_density_kg_m3 / rho_sea_level);
}

} // namespace Thermodynamics
} // namespace FiaPhy

#endif // FIAPHY_THERMODYNAMICS_H
