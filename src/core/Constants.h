/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

/**
 * @file Constants.h
 * @brief Physical constants and configuration defaults for FiaPhy
 * 
 * All constants derived from established meteorological and thermodynamic literature:
 * - Gas constants from NIST
 * - Magnus formula coefficients from Alduchov & Eskridge (1996)
 * - Kasten-Czeplak parameters from solar engineering standards
 */

#ifndef FIAPHY_CONSTANTS_H
#define FIAPHY_CONSTANTS_H

namespace FiaPhy {
namespace Constants {

// ============================================================================
// Thermodynamic Constants
// ============================================================================

/// Specific gas constant for dry air [J/(kg·K)]
constexpr float R_DRY_AIR = 287.058f;

/// Specific gas constant for water vapor [J/(kg·K)]
constexpr float R_WATER_VAPOR = 461.495f;

/// Ratio of gas constants (ε = R_d / R_v)
constexpr float EPSILON = 0.622f;

/// Zero Celsius in Kelvin
constexpr float ZERO_CELSIUS = 273.15f;

/// Stefan-Boltzmann constant [W/(m²·K⁴)]
constexpr float STEFAN_BOLTZMANN = 5.67e-8f;

// ============================================================================
// Magnus Formula Constants (Alduchov & Eskridge, 1996)
// Paper Section 2.3: e_s(T) = 6.112 * exp((17.67 * (T - 273.15)) / (T - 29.65))
// where T is in Kelvin
// ============================================================================

/// Magnus formula coefficient A (dimensionless)
constexpr float MAGNUS_A = 17.67f;

/// Magnus formula coefficient B [K] - CRITICAL: Use in formula as (T_kelvin - 29.65)
/// NOT (T_celsius + 243.5) which is a different formulation
constexpr float MAGNUS_B = 29.65f;

/// Magnus formula coefficient C [hPa]
constexpr float MAGNUS_C = 6.112f;

// ============================================================================
// Enthalpy Calculation Constants
// ============================================================================

/// Specific heat of dry air [kJ/(kg·K)]
constexpr float CP_DRY_AIR = 1.006f;

/// Latent heat of vaporization at 0°C [kJ/kg]
constexpr float LATENT_HEAT_VAPORIZATION = 2501.0f;

/// Specific heat of water vapor [kJ/(kg·K)]
constexpr float CP_WATER_VAPOR = 1.86f;

// ============================================================================
// Kasten-Czeplak Cloud Model Constants
// ============================================================================

/// Cloud attenuation coefficient A
constexpr float KASTEN_A = 0.75f;

/// Cloud attenuation exponent B
constexpr float KASTEN_B = 3.4f;

/// Humidity-to-cloud proxy power (typical value)
constexpr float CLOUD_PROXY_EXPONENT = 1.8f;

// ============================================================================
// Solar Constants
// ============================================================================

/// Solar constant (extra-terrestrial irradiance) [W/m²]
constexpr float SOLAR_CONSTANT = 1367.0f;

/// Typical maximum GHI at sea level [W/m²]
constexpr float MAX_GHI_SEA_LEVEL = 1200.0f;

// ============================================================================
// Sensor Physical Properties (BME280 defaults)
// ============================================================================

/// Typical sensor package surface area [m²]
constexpr float SENSOR_AREA_M2 = 6.25e-6f;  // 2.5mm × 2.5mm

/// Typical sensor thermal mass [J/K]
constexpr float SENSOR_HEAT_CAPACITY = 0.5f;

/// Temperature resolution [°C]
constexpr float TEMP_RESOLUTION = 0.01f;

/// Humidity resolution [%]
constexpr float HUMIDITY_RESOLUTION = 0.08f;

/// Pressure resolution [hPa]
constexpr float PRESSURE_RESOLUTION = 0.18f;

// ============================================================================
// Validation Thresholds
// ============================================================================

/// Maximum realistic temperature jump [°C/s]
constexpr float MAX_TEMP_JUMP_C_PER_S = 5.0f;

/// Maximum realistic humidity jump [%/s]
constexpr float MAX_HUMIDITY_JUMP_PERCENT_PER_S = 20.0f;

/// Maximum realistic pressure jump [hPa/s]
constexpr float MAX_PRESSURE_JUMP_HPA_PER_S = 10.0f;

/// Minimum valid temperature [°C]
constexpr float MIN_VALID_TEMP_C = -100.0f;

/// Maximum valid temperature [°C]
constexpr float MAX_VALID_TEMP_C = 100.0f;

/// Minimum valid humidity [%]
constexpr float MIN_VALID_HUMIDITY = 0.0f;

/// Maximum valid humidity [%]
constexpr float MAX_VALID_HUMIDITY = 100.0f;

/// Minimum valid pressure [hPa]
constexpr float MIN_VALID_PRESSURE_HPA = 300.0f;

/// Maximum valid pressure [hPa]
constexpr float MAX_VALID_PRESSURE_HPA = 1200.0f;

// ============================================================================
// INR Filter Default Parameters
// ============================================================================

/// Minimum alpha (high inertia, heavy filtering)
constexpr float INR_ALPHA_MIN = 0.05f;

/// Maximum alpha (low inertia, light filtering)
constexpr float INR_ALPHA_MAX = 0.5f;

/// Default alpha (moderate filtering)
constexpr float INR_ALPHA_DEFAULT = 0.2f;

/// Sensitivity gain for adaptive response
constexpr float INR_SENSITIVITY_GAIN = 0.1f;

/// Circular buffer size for derivative calculation
constexpr uint8_t INR_BUFFER_SIZE = 10;

// ============================================================================
// System Limits
// ============================================================================

/// Maximum number of sensor pairs (Reference + Flux)
constexpr uint8_t MAX_SENSOR_PAIRS = 8;

/// Minimum required sensor pairs (per research paper)
constexpr uint8_t MIN_SENSOR_PAIRS = 1;

/// Frame buffer size (for historical analysis)
constexpr uint8_t FRAME_BUFFER_SIZE = 16;

// ============================================================================
// Calibration Defaults
// ============================================================================

/// Default thermal time constant [seconds]
constexpr float DEFAULT_TAU_S = 30.0f;

/// Default solar absorptivity (black epoxy coating)
constexpr float DEFAULT_ABSORPTIVITY = 0.90f;

/// Default self-heating offset [°C]
constexpr float DEFAULT_SELF_HEATING_C = 0.8f;

/// Convective heat transfer coefficient (still air) [W/(m²·K)]
constexpr float HC_STILL_AIR = 5.0f;

/// Convective heat transfer coefficient (windy) [W/(m²·K)]
constexpr float HC_FORCED_CONV = 25.0f;

// ============================================================================
// Mathematical Constants
// ============================================================================

constexpr float FIAPHY_PI = 3.14159265359f;
constexpr float FIAPHY_TWO_PI = 6.28318530718f;
constexpr float FIAPHY_DEG_TO_RAD = 0.01745329251f;
constexpr float FIAPHY_RAD_TO_DEG = 57.2957795131f;

// ============================================================================
// Fixed-Point Scaling (for 8-bit MCU optimization)
// ============================================================================

/// Q10.6 scaling factor (1024)
constexpr int32_t Q10_SCALE = 1024;

/// Q16.16 scaling factor (65536)
constexpr int32_t Q16_SCALE = 65536;

} // namespace Constants
} // namespace FiaPhy

#endif // FIAPHY_CONSTANTS_H
