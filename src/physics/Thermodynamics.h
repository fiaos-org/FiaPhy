#ifndef FIAPHY_THERMODYNAMICS_H
#define FIAPHY_THERMODYNAMICS_H

#include "../core/Types.h"
#include <math.h>

namespace FiaPhy {
namespace Thermodynamics {


inline float saturationVaporPressure(float tempc) {
    float tempk = tempc + 273.15f;
    
    float numerator = 17.67f * (tempk - 273.15f);
    float denominator = tempk - 29.65f;
    
    return 6.112f * expf(numerator / denominator);
}


inline float vaporPressure(float tempc, float humidity) {
    float esat = saturationVaporPressure(tempc);
    return esat * (humidity / 100.0f);
}


inline float airDensity(float pressure, float tempc, float humidity) {
    float tempk = tempc + 273.15f;
    float vaporhpa = vaporPressure(tempc, humidity);
    
    float pressurepa = pressure * 100.0f;
    float vaporpa = vaporhpa * 100.0f;

    float drydensity = (pressurepa - vaporpa) /
                       (Constants::rdryair * tempk);

    float vapordensity = vaporpa /
                         (Constants::rwatervapor * tempk);
    
    return drydensity + vapordensity;
}


inline float mixingRatio(float pressure, float vaporpressure) {
    return 0.622f * vaporpressure / (pressure - vaporpressure);
}


inline float specificEnthalpy(float tempc, float mixratio) {
    float hdry = 1.006f * tempc;
    float hvapor = 2501.0f + 1.86f * tempc;
    return hdry + mixratio * hvapor;
}


inline AtmosphericState calculateAtmosphericState(float tempc, float humidity, float pressure) {
    AtmosphericState state;

    state.tempk = tempc + 273.15f;
    state.saturationpressure = saturationVaporPressure(tempc);
    state.vaporpressure = vaporPressure(tempc, humidity);

    state.airdensity = airDensity(pressure, tempc, humidity);

    state.mixratio = mixingRatio(pressure, state.vaporpressure);

    state.enthalpy = specificEnthalpy(tempc, state.mixratio);
    
    return state;
}


inline float convectiveHeatTransferCoefficient(float airdensity) {
    constexpr float hsealevel = 10.0f;
    constexpr float rhosealevel = 1.225f;

    return hsealevel * sqrtf(airdensity / rhosealevel);
}


}
}

#endif
