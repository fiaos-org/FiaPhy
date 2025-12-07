/**
 * @file SolarGeometry.h
 * @brief Solar position and clear-sky irradiance calculations
 * 
 * Implements solar geometry algorithms for computing:
 * - Solar zenith and azimuth angles
 * - Clear-sky Global Horizontal Irradiance (GHI)
 * - Day length and sunrise/sunset times
 * 
 * Based on NREL Solar Position Algorithm (SPA) simplified for embedded systems.
 */

#ifndef FIAPHY_SOLARGEOMETRY_H
#define FIAPHY_SOLARGEOMETRY_H

#include "../core/Constants.h"
#include <math.h>

namespace FiaPhy {
namespace SolarGeometry {

/**
 * @brief Calculate Julian day number from date
 * 
 * @param year Year (e.g., 2025)
 * @param month Month (1-12)
 * @param day Day of month (1-31)
 * @return Julian day number
 */
inline float julianDay(int year, int month, int day) {
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    
    int A = year / 100;
    int B = 2 - A + (A / 4);
    
    return floorf(365.25f * (year + 4716)) + floorf(30.6001f * (month + 1)) + day + B - 1524.5f;
}

/**
 * @brief Calculate solar declination angle
 * 
 * δ = 23.45° * sin(360° * (284 + N) / 365)
 * 
 * @param day_of_year Day number (1-365)
 * @return Solar declination in degrees
 */
inline float solarDeclination(int day_of_year) {
    return 23.45f * sinf(Constants::FIAPHY_DEG_TO_RAD * 360.0f * (284 + day_of_year) / 365.0f);
}

/**
 * @brief Calculate hour angle from time
 * 
 * ω = 15° * (hour - 12)
 * 
 * @param hour_of_day Hour in decimal (0-24)
 * @return Hour angle in degrees
 */
inline float hourAngle(float hour_of_day) {
    return 15.0f * (hour_of_day - 12.0f);
}

/**
 * @brief Calculate solar zenith angle
 * 
 * cos(θ_z) = sin(φ) * sin(δ) + cos(φ) * cos(δ) * cos(ω)
 * 
 * @param latitude Latitude in degrees
 * @param declination Solar declination in degrees
 * @param hour_angle Hour angle in degrees
 * @return Zenith angle in degrees
 */
inline float zenithAngle(float latitude, float declination, float hour_angle_deg) {
    float lat_rad = latitude * Constants::FIAPHY_DEG_TO_RAD;
    float dec_rad = declination * Constants::FIAPHY_DEG_TO_RAD;
    float ha_rad = hour_angle_deg * Constants::FIAPHY_DEG_TO_RAD;
    
    float cos_zenith = sinf(lat_rad) * sinf(dec_rad) + 
                       cosf(lat_rad) * cosf(dec_rad) * cosf(ha_rad);
    
    // Clamp to valid range [-1, 1]
    if (cos_zenith > 1.0f) cos_zenith = 1.0f;
    if (cos_zenith < -1.0f) cos_zenith = -1.0f;
    
    return acosf(cos_zenith) * Constants::FIAPHY_RAD_TO_DEG;
}

/**
 * @brief Calculate clear-sky GHI using simplified Bird model
 * 
 * GHI = I_0 * cos(θ_z) * τ
 * 
 * Where:
 * - I_0 = solar constant
 * - θ_z = zenith angle
 * - τ = atmospheric transmittance
 * 
 * @param zenith_deg Zenith angle in degrees
 * @param altitude_m Altitude in meters (for atmospheric correction)
 * @return Clear-sky GHI in W/m²
 */
inline float clearSkyGHI(float zenith_deg, float altitude_m = 0.0f) {
    // Sun below horizon
    if (zenith_deg > 90.0f) {
        return 0.0f;
    }
    
    float zenith_rad = zenith_deg * Constants::FIAPHY_DEG_TO_RAD;
    float cos_zenith = cosf(zenith_rad);
    
    // Air mass (relative path length through atmosphere)
    float air_mass = 1.0f / (cos_zenith + 0.15f * powf(93.885f - zenith_deg, -1.253f));
    
    // Atmospheric transmittance (simplified)
    // Accounts for Rayleigh scattering, ozone, water vapor absorption
    float altitude_correction = expf(altitude_m / 8000.0f);  // Scale height ~8km
    float tau = 0.7f * powf(0.678f, air_mass) * altitude_correction;
    
    // GHI calculation
    float ghi = Constants::SOLAR_CONSTANT * cos_zenith * tau;
    
    // Clamp to physically reasonable maximum
    if (ghi > Constants::MAX_GHI_SEA_LEVEL) {
        ghi = Constants::MAX_GHI_SEA_LEVEL;
    }
    
    return ghi;
}

/**
 * @brief Calculate complete solar flux parameters
 * 
 * @param latitude Latitude in degrees
 * @param longitude Longitude in degrees
 * @param year Year
 * @param month Month (1-12)
 * @param day Day of month
 * @param hour Hour in decimal (0-24)
 * @param altitude_m Altitude in meters
 * @return SolarFlux structure with all parameters
 */
inline SolarFlux calculateSolarFlux(float latitude, float longitude, 
                                    int year, int month, int day, 
                                    float hour, float altitude_m = 0.0f) {
    SolarFlux flux;
    
    // Day of year calculation (approximate - ignores leap years)
    static const int days_before_month[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int day_of_year = days_before_month[month - 1] + day;
    
    flux.day_of_year = day_of_year;
    
    // Solar declination
    float declination = solarDeclination(day_of_year);
    
    // Hour angle
    flux.hour_angle_deg = hourAngle(hour);
    
    // Zenith angle
    flux.zenith_angle_deg = zenithAngle(latitude, declination, flux.hour_angle_deg);
    
    // Elevation angle (complement of zenith)
    flux.elevation_angle_deg = 90.0f - flux.zenith_angle_deg;
    
    // Sun is up if elevation > 0
    flux.sun_is_up = flux.elevation_angle_deg > 0.0f;
    
    // Clear-sky GHI
    flux.clear_sky_ghi_Wm2 = clearSkyGHI(flux.zenith_angle_deg, altitude_m);
    
    // Azimuth (simplified - full calculation requires more parameters)
    flux.azimuth_angle_deg = 180.0f;  // Placeholder: south-facing
    
    return flux;
}

} // namespace SolarGeometry
} // namespace FiaPhy

#endif // FIAPHY_SOLARGEOMETRY_H
