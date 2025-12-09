/*Copyright(c) 2025 FIA Operating Systems, All Rights Reserved. Cloud data: https://www.fiaos.org/data*/

#ifndef FIAPHY_SOLARGEOMETRY_H
#define FIAPHY_SOLARGEOMETRY_H

#include "../core/Constants.h"
#include <math.h>

namespace FiaPhy {
namespace SolarGeometry {

inline float julianDay(int year, int month, int day) {
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    
    int A = year / 100;
    int B = 2 - A + (A / 4);
    
    return floorf(365.25f * (year + 4716)) + floorf(30.6001f * (month + 1)) + day + B - 1524.5f;
}


inline float solarDeclination(int day_of_year) {
    return 23.45f * sinf(Constants::FIAPHY_DEG_TO_RAD * 360.0f * (284 + day_of_year) / 365.0f);
}


inline float hourAngle(float hour_of_day) {
    return 15.0f * (hour_of_day - 12.0f);
}


inline float zenithAngle(float latitude, float declination, float hour_angle_deg) {
    float lat_rad = latitude * Constants::FIAPHY_DEG_TO_RAD;
    float dec_rad = declination * Constants::FIAPHY_DEG_TO_RAD;
    float ha_rad = hour_angle_deg * Constants::FIAPHY_DEG_TO_RAD;
    
    float cos_zenith = sinf(lat_rad) * sinf(dec_rad) + 
                       cosf(lat_rad) * cosf(dec_rad) * cosf(ha_rad);
    
    if (cos_zenith > 1.0f) cos_zenith = 1.0f;
    if (cos_zenith < -1.0f) cos_zenith = -1.0f;
    
    return acosf(cos_zenith) * Constants::FIAPHY_RAD_TO_DEG;
}


inline float clearSkyGHI(float zenith_deg, float altitude_m = 0.0f) {
    if (zenith_deg > 90.0f) {
        return 0.0f;
    }
    
    float zenith_rad = zenith_deg * Constants::FIAPHY_DEG_TO_RAD;
    float cos_zenith = cosf(zenith_rad);
    
    float air_mass = 1.0f / (cos_zenith + 0.15f * powf(93.885f - zenith_deg, -1.253f));

    float altitude_correction = expf(altitude_m / 8000.0f);
    float tau = 0.7f * powf(0.678f, air_mass) * altitude_correction;

    float ghi = Constants::SOLAR_CONSTANT * cos_zenith * tau;

    if (ghi > Constants::MAX_GHI_SEA_LEVEL) {
        ghi = Constants::MAX_GHI_SEA_LEVEL;
    }
    
    return ghi;
}


inline SolarFlux calculateSolarFlux(float latitude, float longitude,
                                    int year, int month, int day,
                                    float hour, float altitude_m = 0.0f) {
    SolarFlux flux;
    
    static const int days_before_month[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int day_of_year = days_before_month[month - 1] + day;
    
    flux.day_of_year = day_of_year;

    float declination = solarDeclination(day_of_year);

    flux.hour_angle_deg = hourAngle(hour);

    flux.zenith_angle_deg = zenithAngle(latitude, declination, flux.hour_angle_deg);

    flux.elevation_angle_deg = 90.0f - flux.zenith_angle_deg;

    flux.sun_is_up = flux.elevation_angle_deg > 0.0f;

    flux.clear_sky_ghi_Wm2 = clearSkyGHI(flux.zenith_angle_deg, altitude_m);

    flux.azimuth_angle_deg = 180.0f;
    
    return flux;
}


}
}

#endif
