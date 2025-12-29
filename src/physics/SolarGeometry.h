#ifndef FIAPHY_SOLARGEOMETRY_H
#define FIAPHY_SOLARGEOMETRY_H

#include "../core/Constants.h"
#include <math.h>

namespace FiaPhy {
namespace SolarGeometry {

inline float  julianDay(int year, int month, int day) {
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    
    int A  = year / 100;
    int B = 2 - A + (A / 4);
    
    return floorf(365.25f * (year + 4716)) + floorf(30.6001f * (month + 1)) + day + B - 1524.5f;
}


inline float solarDeclination(int dayofyear) {
    return 23.45f * sinf(Constants::degtorad * 360.0f * (284 + dayofyear) / 365.0f);
}


inline float hourAngle(float hourofday) {
    return 15.0f * (hourofday - 12.0f);
}


inline float zenithAngle(float latitude, float declination, float hourangle) {
    float latrad = latitude * Constants::degtorad;
    float decrad = declination * Constants::degtorad;
    float harad = hourangle * Constants::degtorad;
    
    float coszenith = sinf(latrad) * sinf(decrad) +
                      cosf(latrad) * cosf(decrad) * cosf(harad);
    
    if (coszenith > 1.0f) coszenith = 1.0f;
    if (coszenith < -1.0f) coszenith = -1.0f;
    
    return acosf(coszenith) * Constants::radtodeg;
}


inline float clearSkyGHI(float zenithdeg, float altitude = 0.0f) {
    if (zenithdeg > 90.0f) {
        return 0.0f;
    }
    
    float zenithrad = zenithdeg * Constants::degtorad;
    float coszenith = cosf(zenithrad);
    
    float airmass = 1.0f / (coszenith + 0.15f * powf(93.885f - zenithdeg, -1.253f));

    float altitudecorrection = expf(altitude / 8000.0f);
    float tau = 0.7f * powf(0.678f, airmass) * altitudecorrection;

    float ghi = Constants::solarconstant * coszenith * tau;

    if (ghi > Constants::maxghisealevel) {
        ghi = Constants::maxghisealevel;
    }
    
    return ghi;
}


inline SolarFlux calculateSolarFlux( float latitude, float longitude,
                                    int year, int month, int day,
                                    float hour, float altitude = 0.0f) {
    SolarFlux flux;
    
    static const int daysbeforemonth[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int dayofyear = daysbeforemonth[month - 1] + day;
    
    flux.dayofyear = dayofyear;

    float declination = solarDeclination(dayofyear);

    flux.hourangle = hourAngle(hour);

    flux.zenithangle = zenithAngle(latitude, declination, flux.hourangle);

    flux.elevationangle = 90.0f - flux.zenithangle;

    flux.sunup = flux.elevationangle > 0.0f;

    flux.clearskyghi = clearSkyGHI(flux.zenithangle, altitude);

    flux.azimuthangle = 180.0f;
    
    return flux;
}


}
}

#endif
